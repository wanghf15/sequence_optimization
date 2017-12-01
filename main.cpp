#include <iostream>
#include <vector>
#include <fstream>
#include <map>
#include <glob.h>
#include <chrono>
#include "src/json.h"
#include "src/system_parameters.h"
#include "src/ls_optimization.h"
#include "src/log_error_info.h"

using json = nlohmann::json;
using namespace std;
using namespace sequence_optimization;

std::chrono::high_resolution_clock::time_point inline get_now() {
    return std::chrono::high_resolution_clock::now();
}

double inline get_duration(std::chrono::high_resolution_clock::time_point& start,
                           std::chrono::high_resolution_clock::time_point& end) {
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

vector<string> globVector(const string& pattern) {
    glob_t glob_result;
    glob(pattern.c_str(), GLOB_TILDE, nullptr, &glob_result);

    vector<string> files;
    for(unsigned int i=0; i<glob_result.gl_pathc; ++i){
        files.emplace_back(glob_result.gl_pathv[i]);
    }

    globfree(&glob_result);
    return files;
}

int main(int argc, char** argv) {
    LeastSquareOptimization leastSquareOptimization;
    Log_Error_Info logger;

    double angle_input = 30;
//    CERES_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, *argv, true);
    google::InitGoogleLogging(argv[0]);

    // load data from json
    string dataset_folder = "../data/tracklets/";
    vector<string> json_path_list = globVector(dataset_folder + "*.json");
    string json_path = "../data/tracklets.json";
    ifstream json_file(json_path);
    json json_frame;
    json_file >> json_frame;
    json_file.close();

    map<int, vector<vector<double>>> id_to_carframes;
    map<int, vector<vector<double>>> id_to_carframes_3d;
    map<int, vector<double>> id_to_estimation;
    map<int, vector<double>> id_to_truth;

    for (int clipidx = 0; clipidx < json_path_list.size(); clipidx++) {
        string json_path = json_path_list[clipidx];
        // cout << json_path << " clip index: " << clipidx << endl;
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

            // only eval on forward car
            bool side_view = false;
            bool flat_road = true;
            for (auto &frame : frames) {
                if (track_id == 623385) {
                    int a = 0;
                }
                float gt_yaw = frame["gt_yaw"];
                vector<vector<float>> cuboid_3d = frame["cuboid_3d"];
                double time_stamp = frame["time_stamp"];
                vector<float> gt_velocity = frame["gt_velocity"];

                double angle_range = angle_input / 180.0 * M_PI;
                if (gt_yaw > angle_range && gt_yaw < M_PI_2 + angle_range)
                    side_view = true;
                if (gt_yaw < -angle_range && gt_yaw > -M_PI_2 - angle_range)
                    side_view = true;

                double gt_position[3] = {(cuboid_3d[0][0] + cuboid_3d[1][0] + cuboid_3d[2][0] + cuboid_3d[3][0]) / 4,
                                         (cuboid_3d[0][1] + cuboid_3d[1][1] + cuboid_3d[2][1] + cuboid_3d[3][1]) / 4,
                                         (cuboid_3d[0][2] + cuboid_3d[1][2] + cuboid_3d[2][2] + cuboid_3d[3][2]) / 4};

                double lateral_range = 5.0;
                if (gt_position[0] > lateral_range || gt_position[0] < -lateral_range) {
                    side_view = true;
                }
                double height_range = 0.5;
                if (gt_position[1] > height_range + camera_height || gt_position[1] < camera_height - height_range) {
                    flat_road = false;
                }
            }

            if (side_view || (!flat_road)) {
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
                }
                else {
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
            // tailstock, four points
            vector<vector<double>> tailstock;

            vector<double> depth_truth;
            vector<double> depth_estimation;

            int track_id = it->first;
            vector<vector<double >> cuboid_2d = it->second;
            vector<vector<double >> cuboid_3d = id_to_carframes_3d[track_id];

            int image_cnt = (int) cuboid_2d.size() / 8;
            if (image_cnt < window_length) {
                continue;
            }
            for (int i = 0; i < image_cnt; i++) {
                vector<double> temp;
                // y
                double y = (cuboid_2d[i * 8 + 2][1] + cuboid_2d[i * 8 + 1][1]) / 2 - static_foey;
                temp.push_back(y);
                // w
                double w = (cuboid_2d[i * 8 + 1][0] - cuboid_2d[i * 8 + 2][0] +
                            cuboid_2d[i * 8 + 5][0] - cuboid_2d[i * 8 + 6][0]) / 2;
                // ground truth depth
                double depth = (cuboid_3d[i * 8 + 1][2] + cuboid_3d[i * 8 + 2][2] +
                                cuboid_3d[i * 8 + 5][2] + cuboid_3d[i * 8 + 6][2]) / 4;
                temp.push_back(w);

                if (i >= 9) {
                    depth_truth.push_back(depth);
                }

                tailstock.push_back(temp);
            }


            for (int j = window_length - 1; j < image_cnt; j++) {
//                auto start = get_now();
                depth_estimation.push_back(leastSquareOptimization.getDepthEstimation(tailstock, j));
//                auto end = get_now();
                // time consuming
//                cout << "time_cost : " << get_duration(start, end) << endl;
            }
            id_to_estimation[track_id] = depth_estimation;
            id_to_truth[track_id] = depth_truth;
        }
    }

    logger.showErrorStat(id_to_truth, id_to_estimation);

    return 0;
}

