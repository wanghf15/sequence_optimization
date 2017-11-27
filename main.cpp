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
    DepthResidual(float w, float f) : w_(w), f_(f){};

    template <typename T> bool operator()(const T* const depth,
                                          const T*const real_width, T* residual) const {
        residual[0] = depth[0] - real_width[0] / w_ * f_;
        return true;
    }

private:
    const float w_;
    const float f_;
};

struct RealWidthResidual {
    RealWidthResidual(float hc, float w, float y) : hc_(hc), w_(w), y_(y){};

    template <typename T> bool operator()(const T*const real_width, T* residual) const {
        residual[0] = hc_ * w_ / y_ - real_width[0];
        return true;
    }

private:
    const float hc_;
    const float w_;
    const float y_;
};

struct VelocityResidual {
    template <typename T> bool operator()(const T*const x2, const T*const x1,
                                          const T*const x0, T* residual) const {
        residual[0] = x2[0] - 2.0 * x1[0] + x0[0];
        return true;
    }
};

DEFINE_string(minimizer, "trust_region", "Minimizer type tp use, choices are : line_search & trust region");

float getCurrentDepth(vector<vector<float>>, int end_index) {
    // init car manager
    float focal_len = 623.5383;
    float camera_height = 1.5165;
    float static_foex = 640;
    float static_foey = 360;

    int window_length = 10;




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
    vector<vector<float>> cuboid_2d;
    vector<vector<float>> tailstock;
    vector<vector<float>> gt_velocity;

    // iterate for each image
    for (auto &frame : frames) {
        json cars = frame["cars"];

        // iterate for each car
        for (auto &car : cars) {
//            cout << car["trackid"] << endl;
            int track_id_int = car["trackid"];



            vector<vector<float>> cur = car["cuboid_2d"];

            if (track_id_int == 388889) {
                for (vector<float> i : cur) {
                    cuboid_2d.push_back(i);
                }
            }

            vector<vector<float>> cuboid_3d = car["cuboid_3d"];

        } // end of cars

    } // end of frames


    int image_cnt = cuboid_2d.size() / 8;
    for (int i = 0; i < image_cnt; i++) {
        vector<float> temp;
        // y
        float y = (cuboid_2d[i * 8 + 2][1] - cuboid_2d[i * 8 + 6][1] +
                   cuboid_2d[i * 8 + 1][1] - cuboid_2d[i * 8 + 5][1]) / 2;
        temp.push_back(y);
        // w
        float w = (cuboid_2d[i * 8 + 1][0] - cuboid_2d[i * 8 + 2][0] +
                   cuboid_2d[i * 8 + 5][0] - cuboid_2d[i * 8 + 6][0]) / 2;
        temp.push_back(w);

        tailstock.push_back(temp);
    }

//    for (vector<float> i : gt_velocity) {
//        for (float j : i) {
//            cout << j << ",";
//        }
//        cout << endl;
//    }
//    cout << tailstock.size() << endl;

    vector<float> depth_estimation;
    for (int i = 9; i < image_cnt; i++) {
        depth_estimation.push_back(getCurrentDepth(tailstock, i));
    }

//    double x1 = 3.0;
//    double x2 = -1.0;
//    double x3 = 0.0;
//    double x4 = 1.0;
//
//    Problem problem;
//
////    int window_length = 10;
////    for (int i = 0; i < window_length; i++) {
////        problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
////                new DepthResidual(1 , 2)), NULL, &x1, &x2);
////        problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1, 1>(
////                new RealWidthResidual(1,1,1)), NULL, &x1);
////        problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1>(
////                new VelocityResidual), NULL, &x1);
////    }
//
//    Solver::Options options;
//    LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer, &options.minimizer_type))
//           << "Invalid minimizer : " << FLAGS_minimizer << ", valid options are : trust_region and line_search.";
//
//    options.max_num_iterations = 100;
//    options.linear_solver_type = ceres::DENSE_QR;
//    options.minimizer_progress_to_stdout = true;
//
//    std::cout << "Initial x1 = " << x1 << ", x2 = " << x2
//              << ", x3 = " << x3 << ", x4 = " << x4 << std::endl;
//
//    Solver::Summary summary;
//    Solve(options, &problem, &summary);
//
//    std::cout << summary.FullReport() << "\n";
//    std::cout <<"Final x1 = " << x1
//              << ", x2 = " << x2
//              << ", x3 = " << x3
//              << ", x4 = " << x4 << "\n";
    return 0;
}

