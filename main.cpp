#include <iostream>
#include <vector>
#include <fstream>
#include <map>
#include <glob.h>
#include <opencv2/core/mat.hpp>
#include <opencv/cv.hpp>
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "json.h"
#include "draw_util.h"

using json = nlohmann::json;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace std;
using namespace sequence_optimization;

struct DepthResidual {
    DepthResidual(double w, double f) : w_(w), f_(f){};

    template <typename T> bool operator()(const T* const depth,
                                          const T* const real_width, T* residual) const {
        residual[0] = (depth[0] - real_width[0] / w_ * f_);
        return true;
    }

private:
    const double w_;
    const double f_;
};

struct RealWidthResidual {
    RealWidthResidual(double hc, double w, double y) : hc_(hc), w_(w), y_(y){};

    template <typename T> bool operator()(const T*const real_width, T* residual) const {
        residual[0] = (hc_ * w_ / y_ - real_width[0]);
        return true;
    }

private:
    const double hc_;
    const double w_;
    const double y_;
};

struct VelocityResidual {
    template <typename T> bool operator()(const T*const x2, const T*const x1,
                                          const T*const x0, T* residual) const {
        residual[0] = (x2[0] - 2.0 * x1[0] + x0[0]);
        return true;
    }
};

struct LateralResidual {
    LateralResidual(double x, double y, double hc) : x_(x), y_(y), hc_(hc){};

    template <typename T> bool operator()(const T*const lateral, T* residual) const {
        residual[0] = hc_ / y_ * x_ - lateral[0];
    }

private:
    const double x_;
    const double y_;
    const double hc_;
};

struct TruthObject {
    double x;
    double y;
    double vx;
    double vy;
};

struct EstiObject {
    double depth;
    double vx;
};

DEFINE_string(minimizer, "trust_region", "Minimizer type tp use, choices are : line_search & trust region");

// init car manager
double focal_len = 623.5383;
double camera_height = 1.5165;
double static_foex = 640;
double static_foey = 360;
int window_length = 11;

EstiObject getCurrentDepth(vector<vector<double>> obs, int end_index) {
    // x1~xn, y1~yn, w
    vector<double> optimization_vars;

    for (int i = 0; i < window_length; i++) {
        optimization_vars.push_back(camera_height / (obs[end_index - window_length + 1 + i][0] - static_foey) * focal_len);
        optimization_vars.push_back(camera_height / (obs[end_index - window_length + 1 + i][0] - static_foey) *
                                            (obs[end_index - window_length + 1 + i][2]- static_foex));
    }

    optimization_vars.push_back(camera_height / (obs[end_index - window_length + 1][0] - static_foey)
                                    * obs[end_index - window_length + 1][1]);
    Problem problem;

    for (int i = 0; i < window_length; i++) {
        problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
                new DepthResidual(obs[end_index - window_length + 1 + i][1], focal_len)), NULL,
                                 &optimization_vars[i * 2], &optimization_vars[window_length * 2]);
        problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
                new RealWidthResidual(camera_height, obs[end_index - window_length + 1 + i][1],
                                      obs[end_index - window_length + 1 + i][0])), NULL, &optimization_vars[window_length * 2]);
//        problem.AddResidualBlock(new AutoDiffCostFunction<LateralResidual, 1, 1>(
//                new LateralResidual(obs[end_index - window_length + 1 + i][2],
//                                      obs[end_index - window_length + 1 + i][0], camera_height)), NULL, &optimization_vars[i * 2 + 1]);
        if (i < window_length - 2) {
            problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
                    new VelocityResidual), NULL, &optimization_vars[(i + 2) * 2], &optimization_vars[(i + 1) * 2], &optimization_vars[i * 2]);
            problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
                    new VelocityResidual), NULL, &optimization_vars[(i + 2) * 2 + 1], &optimization_vars[(i + 1) * 2 + 1], &optimization_vars[i * 2 + 1]);
        }
    }

    Solver::Options options;
    LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer, &options.minimizer_type))
           << "Invalid minimizer : " << FLAGS_minimizer << ", valid options are : trust_region and line_search.";

    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    Solver::Summary summary;
    Solve(options, &problem, &summary);

    EstiObject object;
    object.depth = optimization_vars[window_length * 2 - 2];
    object.vx = optimization_vars[window_length * 2 - 2] - optimization_vars[window_length * 2 - 4];

    return object;
}

/**
 * 统计各个区间误差分布，并打印
 * @param truth
 * @param estimation
 */
void showErrorStat(map<int,vector<TruthObject> > truth, map<int, vector<EstiObject> > estimation) {
    double errors[10];
    int error_count[10] = {0};
    map<int, vector<TruthObject> >::iterator it;
    for (it = truth.begin(); it != truth.end(); ++it) {
        int trackid = it->first;
        vector<TruthObject> depth_truth = it->second;
        vector<EstiObject> depth_estimation = estimation[trackid];
        for (int i = 0; i < depth_truth.size(); i++) {
            if (depth_truth[i].x < 10) {
                errors[0] += abs(depth_truth[i].x - depth_estimation[i].depth) / depth_truth[i].x;
                error_count[0]++;
            }
            else if (depth_truth[i].x < 20) {
                errors[1] += abs(depth_truth[i].x - depth_estimation[i].depth) / depth_truth[i].x;
                error_count[1]++;
            }
            else if (depth_truth[i].x < 30) {
                errors[2] += abs(depth_truth[i].x - depth_estimation[i].depth) / depth_truth[i].x;
                error_count[2]++;
            }
            else if (depth_truth[i].x < 40) {
                errors[3] += abs(depth_truth[i].x - depth_estimation[i].depth) / depth_truth[i].x;
                error_count[3]++;
            }
            else if (depth_truth[i].x < 50) {
                errors[4] += abs(depth_truth[i].x - depth_estimation[i].depth) / depth_truth[i].x;
                error_count[4]++;
            }
            else if (depth_truth[i].x < 60) {
                errors[5] += abs(depth_truth[i].x - depth_estimation[i].depth) / depth_truth[i].x;
                error_count[5]++;
            }
            else if (depth_truth[i].x < 70) {
                errors[6] += abs(depth_truth[i].x - depth_estimation[i].depth) / depth_truth[i].x;
                error_count[6]++;
            }
            else {
                errors[7] += abs(depth_truth[i].x - depth_estimation[i].depth) / depth_truth[i].x;
                error_count[7]++;
            }
        }
    }

    for (int i = 0; i < 10; i++) {
        cout << "per error : " << errors[i] / error_count[i] * 100.0 << "%" << endl;
    }
}

vector<string> globVector(const string& pattern){
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
    Utils utils;
    double angle_input = 30;
//    CERES_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, *argv, true);
    google::InitGoogleLogging(argv[0]);

    // load data from json
    string dataset_folder = "../data/tracklet/";
    vector<string> json_path_list = globVector(dataset_folder + "*.json");
    string json_path = "../data/tracklets.json";
    ifstream json_file(json_path);
    json json_frame;
    json_file >> json_frame;
    json_file.close();

    map<int, vector<vector<double>>> id_to_carframes;
    map<int, vector<vector<double>>> id_to_carframes_3d;
    map<int, vector<EstiObject>> id_to_estimation;
    map<int, vector<TruthObject>> id_to_truth;

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

            vector<TruthObject> depth_truth;
            vector<EstiObject> depth_estimation;

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
                temp.push_back(w);
                // x
                double x = (cuboid_2d[i * 8 + 0][0] + cuboid_2d[i * 8 + 1][0] +
                            cuboid_2d[i * 8 + 2][0] + cuboid_2d[i * 8 + 3][0]) / 4;
                temp.push_back(x);

                // ground truth lateral
                double lateral = (cuboid_3d[i * 8 + 1][0] + cuboid_3d[i * 8 + 2][0] +
                                cuboid_3d[i * 8 + 0][0] + cuboid_3d[i * 8 + 3][0]) / 4;


                if (i >= window_length - 1) {
                    // ground truth depth
                    double depth = (cuboid_3d[i * 8 + 1][2] + cuboid_3d[i * 8 + 2][2] +
                                    cuboid_3d[i * 8 + 5][2] + cuboid_3d[i * 8 + 6][2]) / 4;
                    double last_depth = (cuboid_3d[(i-1) * 8 + 1][2] + cuboid_3d[(i-1) * 8 + 2][2] +
                                         cuboid_3d[(i-1) * 8 + 5][2] + cuboid_3d[(i-1) * 8 + 6][2]) / 4;
                    TruthObject info;
                    info.x = depth;
                    info.y = lateral;
                    double velocity_x = depth - last_depth;
                    info.vx = velocity_x;
                    depth_truth.push_back(info);
                }

                tailstock.push_back(temp);
            }

            for (int j = window_length - 1; j < image_cnt; j++) {
                depth_estimation.push_back(getCurrentDepth(tailstock, j));
            }
            id_to_estimation[track_id] = depth_estimation;
            id_to_truth[track_id] = depth_truth;
        }
    }

    showErrorStat(id_to_truth, id_to_estimation);

    // 画图
    const float depth_bot = -2.f;
    const float lateral_half_range = 10.f;
    cv::Mat bvImg = cv::Mat::zeros(800, 300, CV_8UC3);

    Object3d object3d;

    object3d.length_= 5.0;
    object3d.width_ = 2;
    object3d.position_y_ = 2;
    object3d.theta_ = 0;
    for (auto &truth : id_to_truth) {
        int track_id = truth.first;
        vector<TruthObject> cur_truth = truth.second;
        vector<EstiObject> cur_esti = id_to_estimation[track_id];
        for (int i = 0; i < cur_truth.size(); i++) {
            bvImg.setTo(cv::Scalar(0, 0, 0));
            object3d.position_x_ = cur_truth[i].x;
            object3d.track_id_ = -1;
            object3d.speed_x_ = cur_truth[i].vx;
            vector<Object3d> left_lmk;
            left_lmk.push_back(object3d);
            object3d.track_id_ = 1;
            object3d.position_x_ = cur_esti[i].depth;
            object3d.speed_x_ = cur_esti[i].vx;
            vector<Object3d> right_lmk;
            right_lmk.push_back(object3d);
            utils.draw_bird_view(bvImg, left_lmk, depth_bot, lateral_half_range);
            utils.draw_bird_view(bvImg, right_lmk, depth_bot, lateral_half_range);
            char* filename;
            sprintf(filename, "../data/res_img/%d_%d.jpg", track_id, i);
//            cv::imshow("bv", bvImg);
            cv::imwrite(filename, bvImg);
//            cvWaitKey(0);
        }
    }


    return 0;
}

