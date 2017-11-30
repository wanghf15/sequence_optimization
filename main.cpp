#include <iostream>
#include <vector>
#include <fstream>
#include <map>
#include <glob.h>
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "json.h"

using json = nlohmann::json;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace std;

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
        residual[0] = 5.0 * (hc_ * w_ / y_ - real_width[0]);
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
        residual[0] = 10.0 * (x2[0] - 2.0 * x1[0] + x0[0]);
        return true;
    }
};

DEFINE_string(minimizer, "trust_region", "Minimizer type tp use, choices are : line_search & trust region");

// init car manager
double focal_len = 623.5383;
double camera_height = 1.5165;
double static_foex = 640;
double static_foey = 360;
int window_length = 11;

double getCurrentDepth(vector<vector<double>> obs, int end_index) {
    // x1~xn, w
    vector<double> optimization_vars;

    if (end_index == 191) {
        cout << "a" << endl;
    }

    for (int i = 0; i < window_length; i++) {
        optimization_vars.push_back(camera_height / (obs[end_index - window_length + 1 + i][0] - static_foey) * focal_len);
        if (i == window_length - 1) {
            optimization_vars.push_back(camera_height / (obs[end_index - window_length + 1][0] - static_foey)
                                        * obs[end_index - window_length + 1][1]);
        }
    }

    Problem problem;

    for (int i = 0; i < window_length; i++) {
        problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
                new DepthResidual(obs[end_index - window_length + 1 + i][1], focal_len)), NULL,
                                 &optimization_vars[i], &optimization_vars[window_length]);
        problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
                new RealWidthResidual(camera_height, obs[end_index - window_length + 1 + i][1],
                                      obs[end_index - window_length + 1 + i][0])), NULL, &optimization_vars[window_length]);
        if (i < window_length - 2) {
            problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
                    new VelocityResidual), NULL, &optimization_vars[i + 2], &optimization_vars[i + 1], &optimization_vars[i]);
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

//    cout << summary.FullReport();

    return optimization_vars[window_length - 1];
}

/**
 * 统计各个区间误差分布，并打印
 * @param truth
 * @param estimation
 */
void showErrorStat(vector<double> truth, vector<double> estimation) {
    double errors[5];
    int error_count[5] = {0};
    for (int i = 0; i < truth.size(); i++) {
        if (truth[i] < 10) {
            errors[0] += abs(truth[i] - estimation[i]) / truth[i];
            error_count[0]++;
        }
        else if (truth[i] < 20) {
            errors[1] += abs(truth[i] - estimation[i]) / truth[i];
            error_count[1]++;
        }
        else if (truth[i] < 30) {
            errors[2] += abs(truth[i] - estimation[i]) / truth[i];
            error_count[2]++;
        }
        else {
            errors[3] += abs(truth[i] - estimation[i]) / truth[i];
            error_count[3]++;
        }
    }
    cout << "per error 0-10m : " << errors[0] / error_count[0] * 100.0 << "%" << endl;
    cout << "per error 10-20m : " << errors[1] / error_count[1] * 100.0 << "%" << endl;
    cout << "per error 20-30m : " << errors[2] / error_count[2] * 100.0 << "%" << endl;
//    cout << "per error >30m : " << errors[3] / error_count[3] * 100.0 << "%" << endl;
}

int main(int argc, char** argv) {
//    CERES_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, *argv, true);
    google::InitGoogleLogging(argv[0]);

    // load data from json
    string json_path = "/home/wanghf/momenta/1280_720.json";
    ifstream json_file(json_path);
    json json_frame;
    json_file >> json_frame;
    json_file.close();

    json frames = json_frame["frames"];
    vector<vector<double>> cuboid_2d;
    vector<vector<double>> tailstock;
    vector<vector<double>> tailstock_3d;
    vector<vector<double>> gt_velocity;
    vector<vector<double>> cuboid_3d;
    vector<double> true_depth;
    map observation_cars;

    // only eval on forward car
    bool side_view = false;
    bool flat_road = true;

    // iterate for each image
    for (auto &frame : frames) {
        json cars = frame["cars"];
        vector<int> car_ids;

        for (auto &car : cars) {
            car_ids.push_back(car["trackid"]);
        }

        // iterate for each car
        for (auto &car : cars) {
            int track_id_int = car["trackid"];
//            double gt_yaw = car["gt_yaw"];
//            double angle_range = M_PI / 6.0;
//            if ((gt_yaw > angle_range && gt_yaw < M_PI_2 + angle_range) ||
//                    (gt_yaw < -angle_range && gt_yaw > -M_PI_2 - angle_range)) {
//                side_view = true;
//            }

            vector<vector<double>> cur = car["cuboid_2d"];
            vector<vector<double>> cur_3d = car["cuboid_3d"];

            double gt_position[3] = {(cur_3d[0][0] + cur_3d[1][0] + cur_3d[2][0] + cur_3d[3][0]) / 4,
                                     (cur_3d[0][1] + cur_3d[1][1] + cur_3d[2][1] + cur_3d[3][1]) / 4,
                                     (cur_3d[0][2] + cur_3d[1][2] + cur_3d[2][2] + cur_3d[3][2]) / 4};

//            side_view = false;
//            flat_road = true;


            if (track_id_int == 388889) {
                double lateral_range = 5.0;
                if (gt_position[0] > lateral_range || gt_position[0] < -lateral_range) {
                    side_view = true;
                }
                double height_range = 0.5;
                if (gt_position[1] > height_range + camera_height || gt_position[1] < camera_height - height_range) {
                    flat_road = false;
                }

                if (side_view || (!flat_road)) {
                    continue;
                }
                for (vector<double> i : cur) {
                    cuboid_2d.push_back(i);
                }
                for (vector<double> j : cur_3d) {
                    cuboid_3d.push_back(j);
                }
            }

        } // end of cars

    } // end of frames


    int image_cnt = (int) cuboid_2d.size() / 8;
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
            true_depth.push_back(depth);
        }


        tailstock.push_back(temp);
    }

    vector<double> depth_estimation;
    for (int i = window_length - 1; i < image_cnt; i++) {
        depth_estimation.push_back(getCurrentDepth(tailstock, i));
    }

//    for (int i = 0 ; i < true_depth.size(); i++) {
//        cout << "abs error : " << setprecision(2) << true_depth[i] - depth_estimation[i] << ", " << "per error : "
//             << setprecision(2) << (true_depth[i] - depth_estimation[i]) / true_depth[i] * 100.0 << "%" << "," <<
//             true_depth[i] << endl;
//    }

    showErrorStat(true_depth, depth_estimation);

    return 0;
}

