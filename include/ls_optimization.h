//
// Created by wanghf on 17-12-1.
//

#ifndef OPTIMIZATION_LS_OPTIMIZATION_H
#define OPTIMIZATION_LS_OPTIMIZATION_H

#include <vector>
#include <ceres/ceres.h>
#include "json.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "glob.h"
#include "cost_function.h"
#include "ls_optimization.h"

using namespace std;
using json = nlohmann::json;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

namespace sequence_optimization {
    struct MeasurementObject {
        double xi;
        double yi;
        double wi;
        double true_depth;
        double true_width;
        double true_yaw;
    };

    class LeastSquareOptimization {
    public:
        MeasurementObject getEstimationResult(vector<MeasurementObject> obs, int end_index);
        double getEstimationResultDepth(vector<vector<double>> obs, int end_index);

    }; // end of class

    struct CarObject {
        int track_id_;

        // size
        double length_;
        double width_;
        double height_;

        // car attributes
        int cls_type_minor_;

        // location
        double position_x_;
        double position_y_;
        double theta_;
        double pos_x_sigma_;
        double pos_y_sigma_;
        double theta_sigma_;

        // speed
        double speed_x_;
        double speed_y_;
        double speed_x_sigma_;
        double speed_y_sigma_;

        // adas functions
        double ttc_;
        double hmw_;
        bool is_dangerous_;
    };

} // end of namespace

#endif //OPTIMIZATION_LS_OPTIMIZATION_H
