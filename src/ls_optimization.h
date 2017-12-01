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

//DEFINE_string(minimizer, "trust_region", "Minimizer type tp use, choices are : line_search & trust region");

namespace sequence_optimization {

    class LeastSquareOptimization {

    public:
        double getDepthEstimation(vector<vector<double>> obs, int end_index);

    }; // end of class

} // end of namespace


#endif //OPTIMIZATION_LS_OPTIMIZATION_H
