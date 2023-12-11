#include "pre_for_small.h"

struct para {
  double pi = 3.1415;
  double t = 0.1;
  double r = 1;
  double omega = pi / 3;
} para;

double get_pos(double theta) {
  /*对于极坐标系*/
  theta += para.omega * para.t;
  return theta;
}

bool get_pos2(double &x, double &y) {
  /* 对于笛卡尔坐标系*/
  double theta = get_pos(atan(y / x));
  x = para.r * cos(theta);
  y = para.r * sin(theta);
  return 0;
}
