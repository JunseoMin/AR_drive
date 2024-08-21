#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>

class CubicSpline1D {
public:
  CubicSpline1D(const std::vector<double>& s, const std::vector<double>& y) {
    int n = s.size();
    h.resize(n - 1);
    b.resize(n - 1);
    u.resize(n - 1);
    v.resize(n - 1);
    z.resize(n);
    a.resize(n);
    c.resize(n);
    d.resize(n);

    for (int i = 0; i < n - 1; ++i) {
      h[i] = s[i + 1] - s[i];
      b[i] = (y[i + 1] - y[i]) / h[i];
    }

    v[0] = 0.0;
    u[0] = 0.0;

    for (int i = 1; i < n - 1; ++i) {
      v[i] = 2 * (h[i] + h[i - 1]);
      u[i] = 6 * (b[i] - b[i - 1]);
    }

    v[n - 1] = 0.0;
    u[n - 1] = 0.0;

    z[0] = 0.0;
    z[n - 1] = 0.0;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n - 2, n - 2);
    Eigen::VectorXd B = Eigen::VectorXd::Zero(n - 2);

    for (int i = 0; i < n - 2; ++i) {
      A(i, i) = v[i + 1];
      if (i != 0) A(i, i - 1) = h[i];
      if (i != n - 3) A(i, i + 1) = h[i + 1];
      B(i) = u[i + 1];
    }

    Eigen::VectorXd Z = A.colPivHouseholderQr().solve(B);

    for (int i = 1; i < n - 1; ++i) {
      z[i] = Z(i - 1);
    }

    for (int i = 0; i < n - 1; ++i) {
      a[i] = y[i];
      c[i] = (z[i + 1] - z[i]) / (6 * h[i]);
      b[i] = z[i] / 2;
      d[i] = (y[i + 1] - y[i]) / h[i] - (2 * h[i] * z[i] + h[i] * z[i + 1]) / 6;
    }
  }

  double calc_position(double s) const {
    int n = h.size();
    int i = n - 1;
    for (int j = 0; j < n - 1; ++j) {
      if (s <= h[j]) {
        i = j;
        break;
      }
    }

    double ds = s - h[i];
    return a[i] + b[i] * ds + c[i] * ds * ds + d[i] * ds * ds * ds;
  }

  double calc_first_derivative(double s) const {
    int n = h.size();
    int i = n - 1;
    for (int j = 0; j < n - 1; ++j) {
        if (s <= h[j]) {
            i = j;
            break;
        }
    }
    double ds = s - h[i];
    return b[i] + 2 * c[i] * ds + 3 * d[i] * ds * ds;
  }

  double calc_second_derivative(double s) const {
    int n = h.size();
    int i = n - 1;
    for (int j = 0; j < n - 1; ++j) {
        if (s <= h[j]) {
            i = j;
            break;
        }
    }
    double ds = s - h[i];
    return 2 * c[i] + 6 * d[i] * ds;
  }

private:
  std::vector<double> h, b, u, v, z, a, c, d;
};

class CubicSpline2D {
public:
  CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y)
    : sx(calc_s(x, y), x), sy(calc_s(x, y), y) {
    s = calc_s(x, y);
  }

  std::vector<double> calc_s(const std::vector<double>& x, const std::vector<double>& y) {
    int n = x.size();
    std::vector<double> s(n, 0.0);
    for (int i = 1; i < n; ++i) {
      double dx = x[i] - x[i - 1];
      double dy = y[i] - y[i - 1];
      s[i] = s[i - 1] + std::hypot(dx, dy);
    }
    return s;
  }

  std::pair<double, double> calc_position(double s) {
    double x = sx.calc_position(s);
    double y = sy.calc_position(s);
    return {x, y};
  }

  double calc_curvature(double s) {
    double dx = sx.calc_first_derivative(s);
    double dy = sy.calc_first_derivative(s);
    double ddx = sx.calc_second_derivative(s);
    double ddy = sy.calc_second_derivative(s);
    return (ddy * dx - ddx * dy) / std::pow(dx * dx + dy * dy, 1.5);
  }

  double calc_yaw(double s) {
    double dx = sx.calc_first_derivative(s);
    double dy = sy.calc_first_derivative(s);
    return std::atan2(dy, dx);
  }

private:
  std::vector<double> s;
  CubicSpline1D sx, sy;
};
