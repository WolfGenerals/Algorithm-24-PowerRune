#ifndef TAYLOR_HPP
#define TAYLOR_HPP
#include "Alias.hpp"


class QuadraticFunctions {
public:
    // 系数
    // f(x) = a*x^2 + b*x + c
    double a, b, c;

    QuadraticFunctions(const double a, const double b, const double c)
        : a(a),
          b(b),
          c(c) {
    }

    double operator()(const double x) const {
        return a * x * x + b * x + c;
    }

    [[nodiscard]] bool valid() const {
        return a!= 0 || b!= 0 || c!= 0;
    }

    static QuadraticFunctions fit(const std::deque<double>& xs, const std::deque<double>& ys) {
        if (xs.size() != ys.size())
            throw std::runtime_error("Taylor::fit: x.size()!= y.size()");
        const auto size = static_cast<double>(xs.size());

        // \begin{cases} \hat a = \cfrac{(\overline{xy}-\overline{x} \cdot \overline{y}) (\overline{x^3}-\overline{x} \cdot \overline{x^2})- (\overline{x^2y}-\overline{x^2}\cdot \overline{y}) (\overline{x^2}-(\overline{x})^2)} {(\overline{x^3}-\overline{x} \cdot \overline{x^2})^2- (\overline{x^4}-(\overline{x^2})^2 \cdot \overline{x^2}) (\overline{x^2}-(\overline{x})^2)} \\ \hat b = \cfrac{\overline{xy}-\overline{x} \cdot \overline{y}- \hat{a}(\overline{x^3}-\overline{x} \cdot \overline{x^2})} {\overline{x^2}-(\overline{x})^2} \\ \hat c = \overline{y} - \hat{a}\overline{x^2} - \hat{b}\overline{x} \end{cases}
        double x   = 0;
        double x2  = 0;
        double x3  = 0;
        double x4  = 0;
        double y   = 0;
        double xy  = 0;
        double x2y = 0;
        for (int i = 0; i < xs.size(); ++i) {
            x += xs[i];
            x2 += xs[i] * xs[i];
            x3 += xs[i] * xs[i] * xs[i];
            x4 += xs[i] * xs[i] * xs[i] * xs[i];
            y += ys[i];
            xy += xs[i] * ys[i];
            x2y += xs[i] * xs[i] * ys[i];
        }
        x /= size;
        x2 /= size;
        x3 /= size;
        x4 /= size;
        y /= size;
        xy /= size;
        x2y /= size;
        double a, b, c;

        a = (xy - x * y) * (x3 - x * x2) - (x2y - x2 * y) * (x2 - x * x);
        a /= (x3 - x * x2) * (x3 - x * x2) - (x4 - x2 * x2) * (x2 - x * x);
        b = (xy - x * y - a * (x3 - x2 * x)) / (x2 - x * x);
        c = y - a * x2 - b * x;

        if (std::isnan(a) || std::isnan(b) || std::isnan(c))
            return {0, 0, 0};
        return {a, b, c};
    }
};


#endif //TAYLOR_HPP
