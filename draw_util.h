//
// Created by wanghf on 17-11-30.
//

#ifndef OPTIMIZATION_DRAW_UTIL_H
#define OPTIMIZATION_DRAW_UTIL_H

#include <vector>
#include <opencv2/core/mat.hpp>

namespace sequence_optimization {

    struct Object3d {
        int track_id_;

        // size
        double length_;
        double width_;
        double height_;

        // car attributes
        int cls_type_minor_;

        // location
        double position_x_;
        double position_y_;
        double theta_;
        double pos_x_sigma_;
        double pos_y_sigma_;
        double theta_sigma_;

        // speed
        double speed_x_;
        double speed_y_;
        double speed_x_sigma_;
        double speed_y_sigma_;

        // adas functions
        double ttc_;
        double hmw_;
        bool is_dangerous_;
    };

    class Utils {
    public:
        void draw_bird_view(cv::Mat visImg, const std::vector<Object3d> &objs,
        const float depth_bot, const float half_lateral_range);
    };


}


#endif //OPTIMIZATION_DRAW_UTIL_H
