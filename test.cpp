#include "trajectory.hpp"
#include <iostream>
#include <glm/gtx/string_cast.hpp>

using namespace std;

int main(void) {
  double earth_mu = 3.986004418E5;
  double earth_r = 6371;

  double r_peri = earth_r+180;
  double ecc = 1.01;
  double a = ((ecc==1)?r_peri:(r_peri/(1-ecc)));

  OrbitalElements oe{ecc, a, 0,0,0,0};

  double epoch = 0.0;

  while (epoch < 10) {
    auto sv = toStateVector(oe, earth_mu, epoch);
    double alt = glm::length(sv.r)-earth_r;
    double vel = glm::length(sv.v);
    cout << epoch << "s :" << glm::to_string(sv.r) << " " << glm::to_string(sv.v) << endl;
    auto oe2 = toOrbitalElements(sv, earth_mu, epoch);
    cout << alt << " " << vel << endl;
    cout << oe2.e << " " << oe2.a << " " << oe2.i << " " << oe2.an << " " << oe2.arg << " " << oe2.m0 << endl;
    epoch += 1;
  }
}