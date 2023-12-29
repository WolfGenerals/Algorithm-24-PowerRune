#include <vector>
enum quadrant { Q1, Q2, Q3, Q4 };
enum rotationDirection { COUNTERCLOCKWISE, CLOCKWISE };

struct point {
  double x, y;
  quadrant q;
  void set_quadrant() {
    if (x > 0 && y > 0) {
      q = Q1;
    } else if (x < 0 && y > 0) {
      q = Q2;
    } else if (x < 0 && y < 0) {
      q = Q3;
    } else if (x > 0 && y < 0) {
      q = Q4;
    }
  }
};

std::vector<point> p;

bool checkRotationDirection() {
  int sum = 0;
  int n = p.size();  // 点的个数
  for (int i = 1; i < p.size(); i++) {
    p[i].set_quadrant();
    if (p[i].q != p[i - 1].q) {
      if (p[i].q == Q1 && p[i - 1].q == Q4) {
        sum++;
      } else if (p[i].q == Q4 && p[i - 1].q == Q1) {
        sum--;
      } else {
        sum += p[i].q > p[i - 1].q ? 1 : -1;
      }
    } else {
      switch (p[i].q) {
        case Q1:
          if (p[i].x > p[i - 1].x && p[i].y > p[i - 1].y) {
            sum++;
          }
          break;
        case Q2:
          if (p[i].x < p[i - 1].x && p[i].y > p[i - 1].y) {
            sum++;
          }
          break;
        case Q3:
          if (p[i].x < p[i - 1].x && p[i].y < p[i - 1].y) {
            sum++;
          }
          break;
        case Q4:
          if (p[i].x > p[i - 1].x && p[i].y < p[i - 1].y) {
            sum++;
          }
          break;
        default:
          break;
      }
    }
  }
  return sum > 0 ? CLOCKWISE : COUNTERCLOCKWISE;
}
