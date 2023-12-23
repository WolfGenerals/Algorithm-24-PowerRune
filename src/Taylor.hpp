#ifndef TAYLOR_HPP
#define TAYLOR_HPP
#include "Alias.hpp"


/**
 * \brief 使用一阶泰勒展开在0附近拟合向量运动
 * vec = vec0+vec1*x
 */
class TaylorVec3 {
    Vec3 vec0;
    Vec3 vec1;

    TaylorVec3(const Vec3& vec0, const Vec3& vec1)
    : vec0(vec0),
        vec1(vec1){}

public:
    /**
     * \brief 使用最小二乘法拟合泰勒展开
     */
    static TaylorVec3 fit(std::vector<Vec3>& ys, std::vector<double> xs) {
        double x_avg = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
        Vec3 y_avg = std::accumulate(ys.begin(), ys.end(), Vec3(0, 0, 0)) / static_cast<double>(ys.
            size());

        Vec3 vec1 = Vec3(0, 0, 0);
        Vec3 vec0 = y_avg;

        cv::solve()
        return {vec0, vec1};

    }

    Vec3 operator()(double x) { return vec0 + vec1 * x + vec2 * x * x; }
};


#endif //TAYLOR_HPP
