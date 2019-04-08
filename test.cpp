#include "trajectory.hpp"
#include <iostream>
#include <glm/gtx/string_cast.hpp>

using namespace std;

int main(void) {
  double earth_mu = 3.986004418E5;
  double earth_r = 6371;
  OrbitalElements oe{5.0, -50000, 0,0,0,0};

  double epoch = 0.0;

  while (epoch < 60000) {
    auto sv = toStateVector(oe, earth_mu, epoch);
    double alt = glm::length(sv.p)-earth_r;
    double vel = glm::length(sv.v);
    //cout << epoch << "s :" << glm::to_string(sv.p) << " " << glm::to_string(sv.v) << endl;
    cout << alt << " " << vel << endl;
    epoch += 1800;
  }
}