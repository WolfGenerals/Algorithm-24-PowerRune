#include "pre_bullet.h"

using std::cos;
using std::sin;
using std::tan;
struct Para {
  double m = 0.5, x = 10, y = 5;
  const double g = 9.8, k = 2, v0 = 20, pi = 3.14159, lr = 10;
  double g_divide_v0 = g / v0;
} para;

double f(double alpha) {
  double ans;
  ans = para.x *
            (tan(alpha) + (para.g * para.m) / (para.k * para.v0 * cos(alpha))) +
        (para.g * para.m * para.m *
         log(1 - (para.k * para.x) / (para.m * para.v0 * cos(alpha)))) /
            (para.k * para.k);
  return ans;
}

double df(double alpha) {
  double ans;
  double x = para.x, g = para.g, m = para.m, k = para.k, v0 = para.v0;
  ans = x * (tan(alpha) * tan(alpha) +
             (g * m * sin(alpha)) / (k * v0 * cos(alpha) * cos(alpha)) + 1) +
        (g * m * x * sin(alpha)) / (k * v0 * cos(alpha) * cos(alpha) *
                                    ((k * x) / (m * v0 * cos(alpha)) - 1));
  return ans;
}

double get_pitch(double x, double y) {
  /*描述有阻力的情况 by myself*/
  para.x = x;
  para.y = y;
  double alpha = para.pi / 4;

  for (int i = 0; i < para.lr; i++) {
    alpha = alpha - (f(alpha) - y) / df(alpha);
  }

  return alpha;
}

double check0(double x, double y, double theta) {
  return y - x * tan(theta) +
         para.g * x * x / (2 * para.v0 * para.v0 * cos(theta) * cos(theta));
}

bool get_pitch0(double x, double y, double &theta) {
  /*描述无阻力的情况 by myself*/
  double Delta = 1 - 2 * y * para.g_divide_v0 -
                 para.g_divide_v0 * para.g_divide_v0 * x * x;
  if (Delta < 0) return 0;
  Delta = std::sqrt(Delta);
  double tan_theta2 = para.v0 * (1 - Delta) / para.g / x;
  theta = atan(tan_theta2);

  return 1;
}

bool get_pitch1(double x, double y, double &theta) {
  /*描述无阻力的情况 from
   * https://sourcelizi.github.io/202309/ballistic-algorithm/  */
  double phi = atan(y / x);
  double l = sqrt(x * x + y * y);
  double alpha = asin((y + para.g_divide_v0 * x * x / para.v0) / l);
  theta = (alpha + phi) / 2;
  return 1;
}
