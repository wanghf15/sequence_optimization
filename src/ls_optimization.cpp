//
// Created by wanghf on 17-12-1.
//

#include "../include/ls_optimization.h"
#include "../include/system_parameters.h"

namespace sequence_optimization {
    /**
     * 优化计算
     * @param obs {x, y, w}
     * @param end_index
     * @return  {x, y, w}
     */
    vector<double> LeastSquareOptimization::getEstimationResult(vector<vector<double>> obs, int end_index) {
        // x1,y1,...,xn,yn,w
        double optimization_vars[window_length * 2 + 1];

        for (int i = 0; i < window_length; i++) {
            // initial xi
            optimization_vars[2 * i] = camera_height / (obs[end_index - window_length + 1 + i][1] - static_foey) * focal_len;
            // initial yi
            optimization_vars[2 * i + 1] = camera_height / (obs[end_index - window_length + 1 + i][1] * obs[end_index - window_length + 1 + i][0]);
        }
        // initial w
        optimization_vars[2 * window_length] = camera_height / (obs[end_index - window_length + 1][1] - static_foey)
                            * obs[end_index - window_length + 1][2];

        double *W = &optimization_vars[window_length * 2];
        Problem problem;

        for (int i = 0; i < window_length; i++) {
            double *Xi = &optimization_vars[i * 2];
            double *Yi = &optimization_vars[i * 2 + 1];
            double xi = obs[end_index - window_length + 1 + i][0];
            double yi = obs[end_index - window_length + 1 + i][1];
            double wi = obs[end_index - window_length + 1 + i][2];

            // 优化变量不能用引用！
            problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
                    new DepthResidual(wi, focal_len)), NULL, Xi, W);

            problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
                    new RealWidthResidual(camera_height, wi, yi)), NULL, W);

//            problem.AddResidualBlock(new AutoDiffCostFunction<LateralDisResidual, 1, 1>(
//                    new LateralDisResidual(xi, yi, camera_height)), NULL, &optimization_vars[i * 2 + 1]);

//            problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidualEx, 1, 1>(
//                    new DepthResidualEx(camera_height, obs[end_index - window_length + 1 + i][0], focal_len)), NULL,
//                                     &optimization_vars[i]);

            if (i < window_length - 2) {
                problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
                        new VelocityResidual), NULL, &optimization_vars[(i + 2) * 2], &optimization_vars[(i + 1) * 2], &optimization_vars[i * 2]);
                problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
                        new VelocityResidual), NULL, &optimization_vars[i * 2 + 5], &optimization_vars[i * 2 + 3], &optimization_vars[i * 2 + 1]);
            }
        }

        Solver::Options options;
        LOG_IF(FATAL, !ceres::StringToMinimizerType("trust_region", &options.minimizer_type))
                << "Invalid minimizer : " << "trust_region" << ", valid options are : trust_region and line_search.";

        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        Solver::Summary summary;
        Solve(options, &problem, &summary);

        vector<double> res;
        res.push_back(optimization_vars[(window_length - 1) * 2]);
        res.push_back(optimization_vars[(window_length - 1) * 2 + 1]);
        res.push_back(optimization_vars[window_length * 2]);

        return res;
    }

    double LeastSquareOptimization::getEstimationResultDepth(vector<vector<double>> obs, int end_index) {
        // x1~xn, w
        vector<double> optimization_vars;

        for (int i = 0; i < window_length; i++) {
            optimization_vars.push_back(camera_height / (obs[end_index - window_length + 1 + i][1] - static_foey) * focal_len);
            if (i == window_length - 1) {
                optimization_vars.push_back(camera_height / (obs[end_index - window_length + 1][1] - static_foey)
                                            * obs[end_index - window_length + 1][2]);
            }
        }

        Problem problem;

        for (int i = 0; i < window_length; i++) {
            problem.AddResidualBlock(new AutoDiffCostFunction<DepthResidual, 1, 1, 1>(
                    new DepthResidual(obs[end_index - window_length + 1 + i][2], focal_len)), NULL,
                                     &optimization_vars[i], &optimization_vars[window_length]);
            problem.AddResidualBlock(new AutoDiffCostFunction<RealWidthResidual, 1, 1>(
                    new RealWidthResidual(camera_height, obs[end_index - window_length + 1 + i][2],
                                          obs[end_index - window_length + 1 + i][1])), NULL, &optimization_vars[window_length]);
            if (i < window_length - 2) {
                problem.AddResidualBlock(new AutoDiffCostFunction<VelocityResidual, 1, 1, 1, 1>(
                        new VelocityResidual), NULL, &optimization_vars[i + 2], &optimization_vars[i + 1], &optimization_vars[i]);
            }
        }

        Solver::Options options;
        LOG_IF(FATAL, !ceres::StringToMinimizerType("trust_region", &options.minimizer_type))
        << "Invalid minimizer : " << "trust_region" << ", valid options are : trust_region and line_search.";

        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        Solver::Summary summary;
        Solve(options, &problem, &summary);

//    cout << summary.FullReport();

        return optimization_vars[window_length - 1];
    }
}

