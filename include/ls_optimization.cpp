//
// Created by wanghf on 17-12-1.
//

#include "../src/ls_optimization.h"
#include "../src/system_parameters.h"

namespace sequence_optimization {
    double LeastSquareOptimization::getDepthEstimation(vector<vector<double>> obs, int end_index) {
        // x1~xn, w
        vector<double> optimization_vars;

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
//            problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidualEx, 1, 1>(
//                    new DepthResidualEx(camera_height, obs[end_index - window_length + 1 + i][0], focal_len)), NULL,
//                                     &optimization_vars[i]);
            if (i < window_length - 2) {
                problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
                        new VelocityResidual), NULL, &optimization_vars[i + 2], &optimization_vars[i + 1], &optimization_vars[i]);
            }
        }

        Solver::Options options;
//        LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer, &options.minimizer_type))
//                << "Invalid minimizer : " << FLAGS_minimizer << ", valid options are : trust_region and line_search.";

        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;

        Solver::Summary summary;
        Solve(options, &problem, &summary);

        return optimization_vars[window_length - 1];
    }

}

