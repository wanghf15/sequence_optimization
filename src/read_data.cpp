//
// Created by wanghf on 17-12-2.
//

#include <fstream>
#include "../include/read_data.h"
#include "glob.h"
#include "../include/json.h"
#include "../include/utils.h"
#include "../include/system_parameters.h"
#include <string>

//using namespace std;
using json = nlohmann::json;

namespace sequence_optimization {
    vector<string> JsonReader::globVector(const string &pattern) {
        glob_t glob_result;
        glob(pattern.c_str(), GLOB_TILDE, nullptr, &glob_result);

        vector<string> files;
        for (unsigned int i = 0; i < glob_result.gl_pathc; ++i) {
            files.emplace_back(glob_result.gl_pathv[i]);
        }

        globfree(&glob_result);
        return files;
    }

    map<int, vector<MeasurementObject>> JsonReader::read_by_tracklets_from_path(string path) {
        map<int, vector<MeasurementObject>> measurements;

        map<int, vector<vector<double >>> id_to_carframes;
        map<int, vector<vector<double >>> id_to_carframes_3d;
        vector<string> json_path_list = globVector(path + "*.json");

        for (int clipidx = 0; clipidx < json_path_list.size(); clipidx++) {
            string json_path = json_path_list[clipidx];
            string json_name = json_path.substr(json_path.find_last_of("/\\") + 1);
            string clip_name = json_name.substr(0, json_name.find_last_of('.'));

            json tracklets;
            ifstream json_file(json_path);
            json_file >> tracklets;
            json_file.close();

            tracklets = tracklets["tracklets"];

            // iterate for each tracklet
            for (auto it = tracklets.begin(); it != tracklets.end(); ++it) {
                string track_id_str = it.key();
                auto &frames = it.value();
                if (track_id_str.length() == 0) {
                    continue;
                }

                int track_id = stoi(track_id_str);

                bool valid_data = true;
                for (auto &frame : frames) {
                    float gt_yaw = frame["gt_yaw"];
                    vector<vector<float>> cuboid_3d = frame["cuboid_3d"];
                    double time_stamp = frame["time_stamp"];
                    vector<float> gt_velocity = frame["gt_velocity"];

                    double gt_position[3] = {
                            (cuboid_3d[0][0] + cuboid_3d[1][0] + cuboid_3d[2][0] + cuboid_3d[3][0]) / 4,
                            (cuboid_3d[0][1] + cuboid_3d[1][1] + cuboid_3d[2][1] + cuboid_3d[3][1]) / 4,
                            (cuboid_3d[0][2] + cuboid_3d[1][2] + cuboid_3d[2][2] + cuboid_3d[3][2]) / 4};

                    if (utils.isInvalidData(gt_yaw, gt_position)) {
                        valid_data = false;
                        break;
                    }
                }
                // data filter
                if (!valid_data) {
                    continue;
                }

                // iterate for each frame
                for (auto &frame : frames) {
                    vector<vector<double>> cur = frame["cuboid_2d"];
                    vector<vector<double>> cur_3d = frame["cuboid_3d"];

                    // first time to observe some car
                    if (id_to_carframes.count(track_id) == 0) {
                        id_to_carframes[track_id] = cur;
                        id_to_carframes_3d[track_id] = cur_3d;
                    } else {
                        vector<vector<double>> tmp = id_to_carframes[track_id];
                        vector<vector<double>> tmp_3d = id_to_carframes_3d[track_id];
                        for (vector<double> i : cur) {
                            tmp.push_back(i);
                        }
                        id_to_carframes[track_id] = tmp;
                        for (vector<double> i : cur_3d) {
                            tmp_3d.push_back(i);
                        }
                        id_to_carframes_3d[track_id] = tmp_3d;
                    }

                } // end of frames

            } // end of tracklets

            // iteration of car map
            map<int, vector<vector<double >>>::iterator it;
            for (it = id_to_carframes.begin(); it != id_to_carframes.end(); ++it) {
                vector<MeasurementObject> cur_measurement;

                vector<vector<double >> estimation;

                int track_id = it->first;
                vector<vector<double >> cuboid_2d = it->second;
                vector<vector<double >> cuboid_3d = id_to_carframes_3d[track_id];

                int image_cnt = (int) cuboid_2d.size() / 8;
//                if (image_cnt < window_length) {
//                    continue;
//                }
                for (int i = 0; i < image_cnt; i++) {
                    MeasurementObject temp;
                    // x
                    double x = (cuboid_2d[i * 8 + 0][0] + cuboid_2d[i * 8 + 1][0] +
                                cuboid_2d[i * 8 + 2][0] + cuboid_2d[i * 8 + 3][0]) / 4 - static_foex;
                    temp.xi = x;
                    // y
                    double y = (cuboid_2d[i * 8 + 2][1] + cuboid_2d[i * 8 + 1][1]) / 2 - static_foey;
                    temp.yi = y;
                    // w
                    double w = (cuboid_2d[i * 8 + 1][0] - cuboid_2d[i * 8 + 2][0] +
                                cuboid_2d[i * 8 + 5][0] - cuboid_2d[i * 8 + 6][0]) / 2;
                    // ground truth depth
                    double depth = (cuboid_3d[i * 8 + 1][2] + cuboid_3d[i * 8 + 2][2] +
                                    cuboid_3d[i * 8 + 5][2] + cuboid_3d[i * 8 + 6][2]) / 4;
                    temp.wi = w;

                    temp.true_depth = depth;

                    cur_measurement.push_back(temp);
                } //end of image

                // 每帧数据包括 x,y,w,true_depth
                measurements[track_id] = cur_measurement;

            } // end of tracklets

        } // end of file

        return measurements;
    }

    map<string, map<int, vector<MeasurementObject >>> JsonReader::read_by_frames_from_path(string path) {
        vector<string> json_path_list = globVector(path + "*.json");

        map<string, map<int, vector<MeasurementObject>>> measurements;

        for (int clipidx = 0; clipidx < json_path_list.size(); clipidx++) {
            string json_path = json_path_list[clipidx];
            string json_name = json_path.substr(json_path.find_last_of("/\\") + 1);
            string clip_name = json_name.substr(0, json_name.find_last_of('.'));

            json frames;
            ifstream json_file(json_path);
            json_file >> frames;
            json_file.close();

            string video_name = frames["video_name"];

            frames = frames["frames"];
            map <int, vector<MeasurementObject>> cur_frame;

            for (auto &frame : frames) {
                json cars = frame["cars"];

                for (auto &car : cars) {
                    vector<MeasurementObject> cur_car;
                    int track_id = car["track_id"];
                    if (cur_frame.count(track_id) == 1) {
                        cur_car = cur_frame[track_id];
                    }
                    vector<vector<double>> cuboid_2d = car["cuboid_2d"];
                    vector<vector<double>> cuboid_3d = car["cuboid_3d"];

                    MeasurementObject temp;
                    // x
                    double x = (cuboid_2d[0][0] + cuboid_2d[1][0] + cuboid_2d[2][0] + cuboid_2d[3][0]) / 4 - static_foex;
                    temp.xi = x;
                    // y
                    double y = (cuboid_2d[2][1] + cuboid_2d[1][1]) / 2 - static_foey;
                    temp.yi = y;
                    // w
                    double w = (cuboid_2d[1][0] - cuboid_2d[2][0] + cuboid_2d[5][0] - cuboid_2d[6][0]) / 2;
                    temp.wi = w;
                    // ground truth depth
                    double depth = (cuboid_3d[1][2] + cuboid_3d[2][2] + cuboid_3d[5][2] + cuboid_3d[6][2]) / 4;
                    temp.true_depth = depth;
                    cur_car.push_back(temp);

                    cur_frame[track_id] = cur_car;
                } // end of cars

            } // end of frames

            measurements[video_name] = cur_frame;

        } // end of file

        return measurements;

    }
}

