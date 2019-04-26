#include "trajectory.hpp"
#include "body.hpp"
#include "math.hpp"

#include <iostream>
#include <vector>
#include <limits>

#include <glm/gtx/string_cast.hpp>

using namespace std;

struct trajectory {
  int body;
  OrbitalElements orbit;
  double end_time;
};

int main(void) {

  System solar;
  auto sun = solar.init(Body(1.32712440018E11));
  auto earth = solar.add(Body(3.986004418E5, 
    OrbitalElements(
      1.711201149295941E-02, 
      1.495724266280181E+08, 
      glm::radians(2.343502100573674E+01), 
      glm::radians(3.599974852472229E+02), 
      glm::radians(1.029016088876174E+02), 
      glm::radians(1.105045521614999E+02))),
    sun);

  double earth_r = 6371;
  vector<trajectory> ship = {{earth, OrbitalElements(0, earth_r+180, 0,0,0,0), std::numeric_limits<double>::infinity()}};

  double epoch = 0.0;

  while (epoch < 10000) {
    trajectory t = ship.at(0);
    if (epoch >= t.end_time) {
      ship.erase(ship.begin());
      continue;
    }
    Body &b = solar.getBody(t.body);

    auto sv = toStateVector(t.orbit, b.getMu(), epoch);
    cout << epoch << endl << "local_state :" << glm::to_string(sv.r()) << " " << glm::to_string(sv.v()) << endl;

    int c = t.body;
    auto abs_sv = sv;
    while (true) {
      int parent = solar.getParent(c);
      if (parent == -1) break;

      auto body_sv = toStateVector(solar.getBody(c).getOrbit(), solar.getBody(parent).getMu(), epoch);
      abs_sv = StateVector(abs_sv.r()+body_sv.r(), abs_sv.v()+body_sv.v());
      c = parent;
    }
    cout << "abs_state : " << glm::to_string(abs_sv.r()) << " " << glm::to_string(abs_sv.v()) << endl;
    cout << "abs mag : " << glm::length(abs_sv.r()) << " : " << glm::length(abs_sv.v()) << endl;
    epoch += 1000;
  }
}