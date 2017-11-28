#include <iostream>
#include <vector>
#include <fstream>
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
        residual[0] = depth[0] - real_width[0] / w_ * f_;
        return true;
    }

private:
    const double w_;
    const double f_;
};

struct RealWidthResidual {
    RealWidthResidual(double hc, double w, double y) : hc_(hc), w_(w), y_(y){};

    template <typename T> bool operator()(const T*const real_width, T* residual) const {
        residual[0] = hc_ * w_ / y_ - real_width[0];
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
        residual[0] = x2[0] - 2.0 * x1[0] + x0[0];
        return true;
    }
};

DEFINE_string(minimizer, "trust_region", "Minimizer type tp use, choices are : line_search & trust region");

double getCurrentDepth(vector<vector<double>> obs, int end_index) {
    // init car manager
    double focal_len = 623.5383;
    double camera_height = 1.5165;
    double static_foex = 640;
    double static_foey = 360;

    int window_length = 10;

    double x1 = camera_height / (obs[end_index - 9][0] - static_foey) * focal_len;
    double x2 = camera_height / (obs[end_index - 8][0] - static_foey) * focal_len;
    double x3 = camera_height / (obs[end_index - 7][0] - static_foey) * focal_len;
    double x4 = camera_height / (obs[end_index - 6][0] - static_foey) * focal_len;
    double x5 = camera_height / (obs[end_index - 5][0] - static_foey) * focal_len;
    double x6 = camera_height / (obs[end_index - 4][0] - static_foey) * focal_len;
    double x7 = camera_height / (obs[end_index - 3][0] - static_foey) * focal_len;
    double x8 = camera_height / (obs[end_index - 2][0] - static_foey) * focal_len;
    double x9 = camera_height / (obs[end_index - 1][0] - static_foey) * focal_len;
    double x10 = camera_height / (obs[end_index - 0][0] - static_foey) * focal_len;
    double w = camera_height / (obs[end_index - 5][0] - static_foey) * obs[end_index - 5][1];

    Problem problem;
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 9][1], focal_len)), NULL, &x1, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 9][1], obs[end_index - 9][0])), NULL, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
            new VelocityResidual), NULL, &x3, &x2, &x1);
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 8][1], focal_len)), NULL, &x2, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 8][1], obs[end_index - 8][0])), NULL, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
            new VelocityResidual), NULL, &x4, &x3, &x2);
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 7][1], focal_len)), NULL, &x3, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 7][1], obs[end_index - 7][0])), NULL, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
            new VelocityResidual), NULL, &x5, &x4, &x3);
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 6][1], focal_len)), NULL, &x4, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 6][1], obs[end_index - 6][0])), NULL, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
            new VelocityResidual), NULL, &x6, &x5, &x4);
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 5][1], focal_len)), NULL, &x5, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 5][1], obs[end_index - 5][0])), NULL, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
            new VelocityResidual), NULL, &x7, &x6, &x5);
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 4][1], focal_len)), NULL, &x6, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 4][1], obs[end_index - 4][0])), NULL, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
            new VelocityResidual), NULL, &x8, &x7, &x6);
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 3][1], focal_len)), NULL, &x7, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 3][1], obs[end_index - 3][0])), NULL, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
            new VelocityResidual), NULL, &x9, &x8, &x7);
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 2][1], focal_len)), NULL, &x8, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 2][1], obs[end_index - 2][0])), NULL, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
            new VelocityResidual), NULL, &x10, &x9, &x8);
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 1][1], focal_len)), NULL, &x9, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 1][1], obs[end_index - 1][0])), NULL, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
            new DepthResidual(obs[end_index - 0][1], focal_len)), NULL, &x10, &w);
    problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
            new RealWidthResidual(camera_height, obs[end_index - 0][1], obs[end_index - 0][0])), NULL, &w);

    Solver::Options options;
    LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer, &options.minimizer_type))
           << "Invalid minimizer : " << FLAGS_minimizer << ", valid options are : trust_region and line_search.";

    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

//    std::cout << "Initial x1 = " << x1 << ", x2 = " << x2
//              << ", x3 = " << x3 << ", x4 = " << x4 << std::endl;

    Solver::Summary summary;
    Solve(options, &problem, &summary);

//    cout << summary.FullReport();

    return x10;
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
    vector<vector<double>> gt_velocity;

    // iterate for each image
    for (auto &frame : frames) {
        json cars = frame["cars"];

        // iterate for each car
        for (auto &car : cars) {
//            cout << car["trackid"] << endl;
            int track_id_int = car["trackid"];



            vector<vector<double>> cur = car["cuboid_2d"];

            if (track_id_int == 388889) {
                for (vector<double> i : cur) {
                    cuboid_2d.push_back(i);
                }
            }

            vector<vector<double>> cuboid_3d = car["cuboid_3d"];

        } // end of cars

    } // end of frames


    int image_cnt = cuboid_2d.size() / 8;
    for (int i = 0; i < image_cnt; i++) {
        vector<double> temp;
        // y
        double y = (cuboid_2d[i * 8 + 2][1] - cuboid_2d[i * 8 + 6][1] +
                   cuboid_2d[i * 8 + 1][1] - cuboid_2d[i * 8 + 5][1]) / 2;
        temp.push_back(y);
        // w
        double w = (cuboid_2d[i * 8 + 1][0] - cuboid_2d[i * 8 + 2][0] +
                   cuboid_2d[i * 8 + 5][0] - cuboid_2d[i * 8 + 6][0]) / 2;
        temp.push_back(w);

        tailstock.push_back(temp);
    }

//    for (vector<double> i : gt_velocity) {
//        for (double j : i) {
//            cout << j << ",";
//        }
//        cout << endl;
//    }
//    cout << tailstock.size() << endl;

    vector<double> depth_estimation;
    for (int i = 9; i < image_cnt; i++) {
        depth_estimation.push_back(getCurrentDepth(tailstock, i));
    }

    for (double i : depth_estimation) {
        cout << i << endl;
    }

    return 0;
}

