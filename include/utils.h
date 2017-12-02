//
// Created by wanghf on 17-12-1.
//

#ifndef OPTIMIZATION_UTILS_H
#define OPTIMIZATION_UTILS_H

//#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>

using namespace std;

namespace sequence_optimization {
    class Utils {
    public:
        // 用角度和横向距离过滤数据
        bool isValidData(double yaw, double position[3]);
        // 获取系统时间
        std::chrono::high_resolution_clock::time_point inline get_now();
        // 计算时间间隔
        double inline get_duration(std::chrono::high_resolution_clock::time_point& start,
                                   std::chrono::high_resolution_clock::time_point& end);

    private:
        double angle_range = 30.0;
        double lateral_range = 5.0;
        double height_range = 0.5;
    };
}

#endif //OPTIMIZATION_UTILS_H
