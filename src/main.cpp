#include <iostream>
#include <vector>
#include <fstream>
#include <map>
#include <glob.h>
#include <chrono>
#include "json.h"
#include "system_parameters.h"
#include "ls_optimization.h"
#include "log_error_info.h"
#include "read_data.h"

using json = nlohmann::json;
using namespace std;
using namespace sequence_optimization;

int main(int argc, char** argv) {
    LeastSquareOptimization leastSquareOptimization;
    JsonReader jsonReader;
    Log_Error_Info logger;

//    CERES_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, *argv, true);
    google::InitGoogleLogging(argv[0]);

    string dataset_folder = "../data/tracklets/";
    map<int, vector<vector<double >>> estimation_result;
    map<int, vector<vector<double >>> measurements = jsonReader.read_by_tracklets_from_path(dataset_folder);
    map<int, vector<double >> ground_truth;

    for (auto it = measurements.begin(); it != measurements.end(); ++it) {
        int track_id = it->first;
        vector<vector<double>> images = it->second;
        vector<vector<double>> cur_result;

        auto image_cnt = images.size();

        for (int i = window_length - 1; i < image_cnt; i++) {
            cur_result.push_back(leastSquareOptimization.getEstimationResult(images, i));
        }

        estimation_result[track_id] = cur_result;
    }

    logger.showErrorStat(ground_truth, estimation_result);

    return 0;
}

