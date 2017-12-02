//
// Created by wanghf on 17-12-2.
//

#include <math.h>
#include "utils.h"
#include "system_parameters.h"

namespace sequence_optimization {
    bool Utils::isValidData(double yaw, double *position) {
        bool side_view = false;
        bool flat_road = true;

        double angle_range = angle_range / 180.0 * M_PI;
        if (yaw > angle_range && yaw < M_PI_2 + angle_range)
            side_view = true;
        if (yaw < -angle_range && yaw > -M_PI_2 - angle_range)
            side_view = true;

        double lateral_range = 5.0;
        if (position[0] > lateral_range || position[0] < -lateral_range) {
            side_view = true;
        }
        double height_range = 0.5;
        if (position[1] > height_range + camera_height || position[1] < camera_height - height_range) {
            flat_road = false;
        }

        return (!flat_road || side_view);
    }

    std::chrono::high_resolution_clock::time_point Utils::get_now() {
        return std::chrono::high_resolution_clock::now();
    }

    double Utils::get_duration(std::chrono::high_resolution_clock::time_point& start,
                               std::chrono::high_resolution_clock::time_point& end) {
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }
}
