//
// Created by wanghf on 17-12-1.
//

#ifndef OPTIMIZATION_UTILS_H
#define OPTIMIZATION_UTILS_H

#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>
#include "ls_optimization.h"

using namespace std;

namespace sequence_optimization {
    class Utils {
    public:
        /**
         * 判断是否在正常角度和横向距离
         * @param yaw 与正前方夹角
         * @param position {x,y,z}
         * @return true: 有效数据
         */
        bool isValidData(double yaw, double position[3]);
        /**
         * 获取系统时间
         * @return
         */
        std::chrono::high_resolution_clock::time_point inline get_now();
        /**
         * 计算时间间隔
         * @param start
         * @param end
         * @return
         */
        double get_duration(std::chrono::high_resolution_clock::time_point& start,
                                   std::chrono::high_resolution_clock::time_point& end);
        /**
         * 画俯视图
         * @param visImg :
         * @param objs
         * @param depth_bot
         * @param half_lateral_range
         * @param cloud
         */
        void draw_bird_view(cv::Mat visImg, const std::vector<sequence_optimization::Object3d> &objs,
                            const float depth_bot, const float half_lateral_range);
    private:
        double angle_range = 30.0;
        double lateral_range = 5.0;
        double height_range = 0.5;
    };
}

#endif //OPTIMIZATION_UTILS_H
