#include "cubic_spline.h"
#include <cmath>
#include <stdexcept>

void CubicSpline::set_points(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size() || x.size() < 2)
        throw std::runtime_error("Invalid input for spline.");
    int n = x.size();
    x_ = x;
    y_ = y;
    a_.resize(n);
    b_.resize(n-1);
    c_.resize(n);
    d_.resize(n-1);

    std::vector<double> h(n-1);
    for (int i = 0; i < n-1; i++) {
        h[i] = x[i+1] - x[i];
    }
    std::vector<double> alpha(n, 0.0);
    for (int i = 1; i < n-1; i++) {
        alpha[i] = (3.0/h[i])*(y[i+1]-y[i]) - (3.0/h[i-1])*(y[i]-y[i-1]);
    }
    std::vector<double> l(n), mu(n), z(n);
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;
    for (int i = 1; i < n-1; i++) {
        l[i] = 2.0*(x[i+1]-x[i-1]) - h[i-1]*mu[i-1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i-1]*z[i-1]) / l[i];
    }
    l[n-1] = 1.0;
    z[n-1] = 0.0;
    c_[n-1] = 0.0;
    for (int j = n-2; j >= 0; j--) {
        c_[j] = z[j] - mu[j]*c_[j+1];
        b_[j] = (y_[j+1] - y_[j]) / h[j] - h[j]*(c_[j+1] + 2*c_[j]) / 3.0;
        d_[j] = (c_[j+1] - c_[j]) / (3.0*h[j]);
        a_[j] = y_[j];
    }
}

double CubicSpline::operator()(double x_val) const {
    int n = x_.size();

    int i = n - 2;
    for (int j = 0; j < n - 1; j++) {
        if (x_val < x_[j+1]) { 
            i = j;
            break;
        }
    }
    double dx = x_val - x_[i];
    return a_[i] + b_[i]*dx + c_[i]*dx*dx + d_[i]*dx*dx*dx;
}

