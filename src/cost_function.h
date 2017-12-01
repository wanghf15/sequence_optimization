//
// Created by wanghf on 17-12-1.
//

#ifndef OPTIMIZATION_COST_FUNCTION_H
#define OPTIMIZATION_COST_FUNCTION_H

struct DepthResidual {
    DepthResidual(double w, double f) : w_(w), f_(f){};

    template <typename T> bool operator()(const T* const depth,
                                          const T* const real_width, T* residual) const {
        residual[0] = (depth[0] - real_width[0] / w_ * f_);
        return true;
    }

private:
    const double w_;
    const double f_;
};

struct RealWidthResidual {
    RealWidthResidual(double hc, double w, double y) : hc_(hc), w_(w), y_(y){};

    template <typename T> bool operator()(const T*const real_width, T* residual) const {
        residual[0] = 5.0 * (hc_ * w_ / y_ - real_width[0]);
        return true;
    }

private:
    const double hc_;
    const double w_;
    const double y_;
};

struct DepthResidualEx {
    DepthResidualEx(double hc, double y, double f) : hc_(hc), y_(y), f_(f){};

    template <typename T> bool operator()(const T*const depth, T* residual) const {
        residual[0] = hc_ / y_ * f_ - depth[0];
    }

private:
    const double hc_;
    const double y_;
    const double f_;
};

struct VelocityResidual {
    template <typename T> bool operator()(const T*const x2, const T*const x1,
                                          const T*const x0, T* residual) const {
        residual[0] = 10.0 * (x2[0] - 2.0 * x1[0] + x0[0]);
        return true;
    }
};

#endif //OPTIMIZATION_COST_FUNCTION_H
