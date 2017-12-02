#include <iostream>
#include <vector>
#include <map>
#include "json.h"
#include "system_parameters.h"
#include "ls_optimization.h"
#include "log_error_info.h"
#include "read_data.h"
#include "utils.h"

using json = nlohmann::json;
using namespace std;
using namespace sequence_optimization;

int main(int argc, char** argv) {
    LeastSquareOptimization leastSquareOptimization;
    JsonReader jsonReader;
    Log_Error_Info logger;
    Utils utils;

    string dataset_folder = "../data/tracklets/";
    map<int, vector<vector<double >>> estimation_result;
    map<int, vector<vector<double >>> measurements = jsonReader.read_by_tracklets_from_path(dataset_folder);
    map<int, vector<double >> ground_truth;

    for (auto it = measurements.begin(); it != measurements.end(); ++it) {
        int track_id = it->first;
        vector<vector<double>> images = it->second;
        vector<vector<double>> cur_result;

        vector<double> cur_truth;

        auto image_cnt = images.size();

        for (int i = window_length - 1; i < image_cnt; i++) {
//            cur_result.push_back(leastSquareOptimization.getEstimationResult(images, i));
//            vector<double> cur_res;
            cur_result.push_back(leastSquareOptimization.getEstimationResult(images, i));
//            cur_result.push_back(cur_res);
            cur_truth.push_back(images[i][3]);
        }

        estimation_result[track_id] = cur_result;
        ground_truth[track_id] = cur_truth;
    }

    logger.showErrorStat(ground_truth, estimation_result);

    //画图
    const float depth_bot = -2.f;
    const float lateral_half_range = 10.f;
    cv::Mat bvImg = cv::Mat::zeros(800, 300, CV_8UC3);
    Object3d object3d;
    object3d.length_= 5.0;
    object3d.width_ = 2;
    object3d.position_y_ = 2;
    object3d.position_x_ = 20;
    object3d.theta_ = 0.1;
    vector<Object3d> left_lmk;
    left_lmk.push_back(object3d);
    vector<Object3d> right_lmk;
    utils.draw_bird_view(bvImg, left_lmk, depth_bot, lateral_half_range);
    cv::imshow("bv", bvImg);
    cvWaitKey(0);

    return 0;
}

