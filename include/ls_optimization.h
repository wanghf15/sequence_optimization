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

using namespace std;
using json = nlohmann::json;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

namespace sequence_optimization {
    class LeastSquareOptimization {
    public:
        vector<double> getEstimationResult(vector<vector<double>> obs, int end_index);
        double getEstimationResultDepth(vector<vector<double>> obs, int end_index);

    }; // end of class

    class Object3d {
    public:
        Object3d();
        int track_id_;

        // size
        float length_;
        float width_;
        float height_;

        // car attributes
        int cls_type_minor_;

        // location
        float position_x_;
        float position_y_;
        float theta_;
        float pos_x_sigma_;
        float pos_y_sigma_;
        float theta_sigma_;

        // speed
        float speed_x_;
        float speed_y_;
        float speed_x_sigma_;
        float speed_y_sigma_;

        // adas functions
        float ttc_;
        float hmw_;
        bool is_dangerous_;
    };

} // end of namespace

#endif //OPTIMIZATION_LS_OPTIMIZATION_H
