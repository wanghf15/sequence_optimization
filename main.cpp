#include <iostream>
#include <vector>
#include <fstream>
#include <glob.h>
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "json.h"

// for convenience
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

int main(int argc, char** argv) {
//    CERES_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, *argv, true);
    google::InitGoogleLogging(argv[0]);

    // init car manager
    float focal_len = 623.5383;
    float camera_height = 1.5165;
    float static_foex = 640;
    float static_foey = 360;

    // load data from json
    string json_path = "/home/wanghf/momenta/1280_720_1.json";
    ifstream json_file(json_path);
    json json_frame;
    json_file >> json_frame;
    json_file.close();

    for (auto it = json_frame.begin(); it != json_frame.end(); ++it) {
        string track_id_str = it.key();
        auto &frames = it.value();
        if (track_id_str.length() == 0) continue;

//        int track_id = stoi(track_id_str);

        // only eval on forward car
        bool side_view = false;
        bool flat_road = true;
        for (auto &frame : frames) {
            float gt_yaw = frame["gt_yaw"];
            vector<vector<float>> cuboid_3d = frame["cuboid_3d"];
            double time_stamp = frame["time_stamp"];
            vector<float> gt_velocity = frame["gt_velocity"];

//            Point3f gt_position = Point3f((cuboid_3d[0][0] + cuboid_3d[1][0] + cuboid_3d[2][0] + cuboid_3d[3][0])/4,
//                                          (cuboid_3d[0][1] + cuboid_3d[1][1] + cuboid_3d[2][1] + cuboid_3d[3][1])/4,
//                                          (cuboid_3d[0][2] + cuboid_3d[1][2] + cuboid_3d[2][2] + cuboid_3d[3][2])/4);
//
//            double lateral_range = 5;
//            if (gt_position.x > lateral_range || gt_position.x < -lateral_range)
//                side_view = true;
//            double height_range = 0.5;
//            if (gt_position.y > cam_param.hei_cam_ + height_range ||
//                gt_position.y < cam_param.hei_cam_ - height_range) {
//                // cout << "gt y: " << gt_position.y << endl;
//                flat_road = false;
//            }
        }
        if (side_view || (!flat_road))
            continue;

        for (auto &frame : frames) {
//            fidx++;

            bool show_img = false;
            bool draw_img = false;

            double time_stamp = frame["time_stamp"];
            // cout << "time stamp: " << std::setprecision(9) << time_stamp << endl;

            string img_name = frame["img_name"];
            float gt_yaw = frame["gt_yaw"];
            // vector<float> gt_position = frame["gt_position"];
            vector<float> gt_velocity = frame["gt_velocity"];
            int yaw_cls = frame["yaw_cls"];
            vector<vector<float>> cuboid_2d = frame["cuboid_2d"];
            vector<vector<float>> cuboid_3d = frame["cuboid_3d"];


            // do your optimization here...

        } // frame


    } // tracklet


    double x1 = 3.0;
    double x2 = -1.0;
    double x3 = 0.0;
    double x4 = 1.0;

    Problem problem;

//    int window_length = 10;
//    for (int i = 0; i < window_length; i++) {
//        problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
//                new DepthResidual(1 , 2)), NULL, &x1, &x2);
//        problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1, 1>(
//                new RealWidthResidual(1,1,1)), NULL, &x1);
//        problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1>(
//                new VelocityResidual), NULL, &x1);
//    }

    Solver::Options options;
    LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer, &options.minimizer_type))
           << "Invalid minimizer : " << FLAGS_minimizer << ", valid options are : trust_region and line_search.";

    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    std::cout << "Initial x1 = " << x1 << ", x2 = " << x2
              << ", x3 = " << x3 << ", x4 = " << x4 << std::endl;

    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";
    std::cout <<"Final x1 = " << x1
              << ", x2 = " << x2
              << ", x3 = " << x3
              << ", x4 = " << x4 << "\n";
    return 0;
}