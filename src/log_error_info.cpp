//
// Created by wanghf on 17-12-1.
//

#include <iostream>
#include <cmath>
#include "../include/log_error_info.h"
#include "ls_optimization.h"


namespace sequence_optimization {
    void Log_Error_Info::showErrorStat(map<int, vector<double> > truth, map<int, vector<MeasurementObject >> estimation) {
        double errors[10];
        int error_count[10] = {0};
        map<int, vector<double> >::iterator it;
        for (it = truth.begin(); it != truth.end(); ++it) {
            int trackid = it->first;
            vector<double> depth_truth = it->second;
            vector<MeasurementObject> depth_estimation = estimation[trackid];
            for (int i = 0; i < depth_truth.size(); i++) {
                if (depth_truth[i] < 10) {
                    errors[0] += abs(depth_truth[i] - depth_estimation[i].xi) / depth_truth[i];
                    error_count[0]++;
                }
                else if (depth_truth[i] < 20) {
                    errors[1] += abs(depth_truth[i] - depth_estimation[i].xi) / depth_truth[i];
                    error_count[1]++;
                }
                else if (depth_truth[i] < 30) {
                    errors[2] += abs(depth_truth[i] - depth_estimation[i].xi) / depth_truth[i];
                    error_count[2]++;
                }
                else if (depth_truth[i] < 40) {
                    errors[3] += abs(depth_truth[i] - depth_estimation[i].xi) / depth_truth[i];
                    error_count[3]++;
                }
                else if (depth_truth[i] < 50) {
                    errors[4] += abs(depth_truth[i] - depth_estimation[i].xi) / depth_truth[i];
                    error_count[4]++;
                }
                else if (depth_truth[i] < 60) {
                    errors[5] += abs(depth_truth[i] - depth_estimation[i].xi) / depth_truth[i];
                    error_count[5]++;
                }
                else if (depth_truth[i] < 70) {
                    errors[6] += abs(depth_truth[i] - depth_estimation[i].xi) / depth_truth[i];
                    error_count[6]++;
                }
                else {
                    errors[7] += abs(depth_truth[i] - depth_estimation[i].xi) / depth_truth[i];
                    error_count[7]++;
                }
            }
        }

        for (int i = 0; i < 10; i++) {
            std::cout << "per error : " << errors[i] / error_count[i] * 100.0 << "%" << endl;
        }
    }
}
