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
    map<int, vector<MeasurementObject>> estimation_result;

    //按tracklet读数据
//    map<int, vector<MeasurementObject>> measurements = jsonReader.read_by_tracklets_from_path(dataset_folder);

    //按frame读数据
    map<string, map<int, vector<MeasurementObject>>> measurements = jsonReader.read_by_frames_from_path("../data/frame/");
    map<int, vector<double >> ground_truth;

//    for (auto it = measurements.begin(); it != measurements.end(); ++it) {
//        int track_id = it->first;
//        vector<MeasurementObject> images = it->second;
//        vector<MeasurementObject> cur_result;
//
//        vector<double> cur_truth;
//
//        auto image_cnt = images.size();
//
//        for (int i = 0; i < image_cnt; i++) {
//            cur_result.push_back(leastSquareOptimization.getEstimationResult(images, i));
//            cur_truth.push_back(images[i].true_depth);
//        }
//
//        estimation_result[track_id] = cur_result;
//        ground_truth[track_id] = cur_truth;
//    }
    for (auto &measurement : measurements) {
        string video_name = measurement.first;
        map<int, vector<MeasurementObject>> cur_measurement = measurement.second;

        for (auto &image : cur_measurement) {
            int track_id = image.first;
            vector<MeasurementObject> cur_image = image.second;


        }
    }

    logger.showErrorStat(ground_truth, estimation_result);

    //画图
    const float depth_bot = -2.f;
    const float lateral_half_range = 10.f;
    cv::Mat bvImg = cv::Mat::zeros(800, 300, CV_8UC3);

    CarObject object3d;

    object3d.length_= 5.0;
    object3d.width_ = 2;
    object3d.position_y_ = 2;
    object3d.theta_ = 0.1;
    for (auto it = ground_truth.begin(); it != ground_truth.end(); ++it) {
        vector<double> test_truth = it->second;
        vector<MeasurementObject> test_estimation = estimation_result[it->first];
        for (int i = 0; i < test_truth.size(); i++) {
            bvImg.setTo(cv::Scalar(0, 0, 0));
            object3d.position_x_ = test_truth[i];
            object3d.track_id_ = -1;
            vector<CarObject> left_lmk;
            left_lmk.push_back(object3d);
            object3d.track_id_ = 1;
            object3d.position_x_ = test_estimation[i].xi;
            vector<CarObject> right_lmk;
            right_lmk.push_back(object3d);
            utils.draw_bird_view(bvImg, left_lmk, depth_bot, lateral_half_range);
            utils.draw_bird_view(bvImg, right_lmk, depth_bot, lateral_half_range);
            cv::imshow("bv", bvImg);
            cvWaitKey(0);
        }
    }


    return 0;
}

