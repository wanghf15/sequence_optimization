//
// Created by wanghf on 17-12-2.
//

#include <math.h>
#include "utils.h"
#include "system_parameters.h"

namespace sequence_optimization {
    bool Utils::isInvalidData(double yaw, double *position) {
        bool side_view = false;
        bool flat_road = true;

        double angle_range_rad = angle_range / 180.0 * M_PI;
        if (yaw > angle_range_rad && yaw < M_PI_2 + angle_range_rad)
            side_view = true;
        if (yaw < -angle_range_rad && yaw > -M_PI_2 - angle_range_rad)
            side_view = true;

        if (position[0] > lateral_range || position[0] < -lateral_range) {
            side_view = true;
        }
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

    inline cv::Point bv_2_visImg(const cv::Point2f &pt_bv, const cv::Point2f &bot_center_bv,
                                 const cv::Point2f &bot_center_img, const double scale)
    {
        cv::Point tmp = (pt_bv - bot_center_bv) * scale;
        return cv::Point(- tmp.y + bot_center_img.x, bot_center_img.y - tmp.x);
    }

    void Utils::draw_bird_view(cv::Mat visImg, const std::vector<sequence_optimization::CarObject> &objs,
                        const float depth_bot, const float half_lateral_range) {
        const double scale = visImg.cols / (half_lateral_range * 2.f);
        const double range_longitude = visImg.rows / scale;
        cv::Point2f bot_center_bv(depth_bot, 0.f);
        cv::Point2f bot_center_img(visImg.cols/2.f, visImg.rows-1.f);

        // add distance ruler
        for (int dist = 0; dist < range_longitude; dist+=10) {
            cv::Point pt2 = bv_2_visImg(cv::Point2f(dist, -half_lateral_range),
                                        bot_center_bv, bot_center_img, scale);
            cv::Point pt1(pt2.x-30, pt2.y);
            cv::line(visImg, pt1, pt2, cv::Scalar(255,255,255), 2);
            pt1 -= cv::Point(15, 10);
            cv::putText(visImg, std::to_string(dist)+"m", pt1, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
        }

        cv::Scalar color;
        // draw car box
        for (int i = 0; i < objs.size(); ++i) {
            const sequence_optimization::CarObject &obj = objs[i];
            const double x0 = obj.position_x_;
            const double y0 = obj.position_y_;
            const double l_half = obj.length_ / 2;
            const double w_half = obj.width_ / 2;
            const double theta = obj.theta_;
            const double costheta = cos(theta);
            const double sintheta = sin(theta);

            if (obj.is_dangerous_) {
                color = cv::Scalar(0, 0, 255);
            } else {
                color = cv::Scalar(0, 255, 0);
            }
            if (obj.track_id_ < 0) {
                color = cv::Scalar(200, 200, 200); //host car
//            color = cv::Scalar(220, 170, 143); //host car
            }

            cv::Point2f p0(x0 + obj.length_, y0 - w_half);
            cv::Point2f p1(x0, y0 - w_half);
            cv::Point2f p2(x0, y0 + w_half);
            cv::Point2f p3(x0 + obj.length_, y0 + w_half);

            cv::Point polygon[1][4];
            polygon[0][0] = bv_2_visImg(p0, bot_center_bv, bot_center_img, scale);
            polygon[0][1] = bv_2_visImg(p1, bot_center_bv, bot_center_img, scale);
            polygon[0][2] = bv_2_visImg(p2, bot_center_bv, bot_center_img, scale);
            polygon[0][3] = bv_2_visImg(p3, bot_center_bv, bot_center_img, scale);
            const cv::Point* ppt[1] = {polygon[0]};
            int npt[] = {4};
            cv::fillPoly(visImg, ppt, npt, 1, color);

            // draw speed vector
            const double vx = obj.speed_x_;
            const double vy = obj.speed_y_;
            cv::line(visImg,
                 bv_2_visImg(cv::Point2f(x0, y0), bot_center_bv, bot_center_img, scale),
                 bv_2_visImg(cv::Point2f(x0 + vx * 2.f, y0 + vy * 2.f), bot_center_bv, bot_center_img, scale),
                 color, 1);

            if (obj.track_id_ >= 0) {
                char out[100];
                sprintf(out, "%.3f", obj.ttc_);
                cv::putText(visImg, out,
                            bv_2_visImg(cv::Point2f(p2.x - 1.2f, p2.y - 0.2f), bot_center_bv, bot_center_img, scale),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            }
            char idstr[100];
            sprintf(idstr, "%d", obj.track_id_);
            cv::putText(visImg, idstr,
                        bv_2_visImg(cv::Point2f(p2.x - 2.5f, p2.y - 0.2f), bot_center_bv, bot_center_img, scale),
                        cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 2);
        }

//        if (!cloud.empty()) {
//            assert(cloud.rows == 3);
//            const double *px = cloud.ptr<double>(0);
//            const double *py = cloud.ptr<double>(1);
//            const double *pz = cloud.ptr<double>(2);
//            for (int j = 0; j < cloud.cols; ++j) {
//                // if (py[j] > 1.0) continue;
//                cv::circle(visImg,
//                           bv_2_visImg(cv::Point2f(pz[j], -px[j]), bot_center_bv, bot_center_img, scale),
//                           1, cv::Scalar(0, 0, 255), -1);
//            }
//        }
    }
}
