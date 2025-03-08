#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <vector>

class CubicSpline {
public:

    void set_points(const std::vector<double>& x, const std::vector<double>& y);


    double operator()(double x_val) const;
private:
    std::vector<double> x_, y_;
    std::vector<double> a_, b_, c_, d_;
};

#endif // CUBIC_SPLINE_H

