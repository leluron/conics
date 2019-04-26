#pragma once

#include <functional>

double find_root_secant(std::function<double(double)> f, double x0, double x1, size_t iterations) {
  for (size_t i=0;i<iterations && x0 != x1;i++) {
    double fx1 = f(x1);
    double x2 = x1 - fx1*(x1-x0)/(fx1-f(x0));
    x0 = x1;
    x1 = x2;
  }
  return x1;
}