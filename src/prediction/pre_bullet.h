#ifndef PRE_BULLET_H
#define PRE_BULLET_H

#include <cmath>
#include <iostream>

struct Para {
  double m = 0.5, x = 10, y = 5;
  const double g = 9.8, k = 2, v0 = 20, pi = 3.14159, lr = 10;
  double g_divide_v0 = g / v0;
};

/**
 * @brief 计算给定坐标点的俯仰角度
 *
 * @param x 横坐标
 * @param y 纵坐标
 * @return double 俯仰角度
 */
double get_pitch(double x, double y); /*描述有阻力的情况 by myself*/

/**
 * 计算给定水平位置和发射角度下子弹的垂直位置，用于检测。
 *
 * @param x 子弹的水平位置。
 * @param y 子弹的垂直位置。
 * @param theta 发射角度（弧度）。
 * @return 子弹的垂直位置。
 */
double check0(double x, double y, double theta);

/*描述无阻力的情况 by myself*/
bool get_pitch0(double x, double y, double &theta);

/*描述无阻力的情况 from
 * https://sourcelizi.github.io/202309/ballistic-algorithm/  */
bool get_pitch1(double x, double y, double &theta);

#endif  // PRE_BULLET_H
