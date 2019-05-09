#include "trajectory.hpp"
#include "body.hpp"
#include "math.hpp"

#include <iostream>
#include <vector>
#include <limits>

#include <glm/gtx/string_cast.hpp>

#include "graphics.hpp"

using namespace std;

enum TrajectoryEnd {
  timedout, crash, soi_leave, soi_enter
};

struct trajectory {
  int body;
  OrbitalElements orbit;
  double end_time;
  TrajectoryEnd end;
};

vector<trajectory> getTrajectories(int body, StateVector sv, const System &system, double epoch, double timeout) {
  if (epoch >= timeout) return {};
  double mu = system.getBody(body).getMu();
  double inf = std::numeric_limits<double>::infinity();
  auto currentTraj = toOrbitalElements(sv, mu, epoch);
  // test crash
  double epochCrash = epochReachAltitude(currentTraj, system.getBody(body).getRadius(), epoch);
  // test soi leave
  double epochSoiLeave = epochReachAltitude(currentTraj, system.getBody(body).getRSOI(), epoch);
  // test soi enter (TODO)
  auto children = system.getChildren(body);
  double epochSoiEnter = inf;

  double nextepoch = std::min(epochCrash, std::min(epochSoiLeave, epochSoiEnter));

  TrajectoryEnd end;
  if (nextepoch == inf) {
    end = timedout;
    nextepoch = timeout;
  } else if (nextepoch == epochCrash) end = crash;
  else if (nextepoch == epochSoiLeave) end = soi_leave;
  else if (nextepoch == epochSoiEnter) end = soi_enter;

  vector<trajectory> next = {};

  if (end == soi_leave) {
    int parent_body = system.getParent(body);
    auto body_orbit = system.getBody(body).getOrbit();
    auto body_sv = toStateVector(body_orbit, epochSoiLeave);
    auto leave_sv = toStateVector(currentTraj, epochSoiLeave);

    next = getTrajectories(parent_body, body_sv+leave_sv, system, epochSoiLeave, timeout);
  } else if (end == soi_enter) {
    // TODO
  }

  trajectory traj = {body, currentTraj, nextepoch, end};
  next.insert(next.begin(), traj);
  return next;
}

void testSoiLeave() {
  double earth_r = 6371;
  double earth_mu = 3.986004418E5;

  System solar;
  auto sun = solar.init(Body(1.32712440018E11, 6.9551E5));
  auto earth = solar.add(Body(earth_mu, earth_r,
    OrbitalElements(
      1.711201149295941E-02, 
      1.495724266280181E+08, 
      glm::radians(2.343502100573674E+01), 
      glm::radians(3.599974852472229E+02), 
      glm::radians(1.029016088876174E+02), 
      glm::radians(1.105045521614999E+02),
      1.32712440018E11)),
    sun);

  double ecc = 1.8;
  double peri = earth_r+180;
  double a = (ecc==1)?peri:(peri/(1-ecc));
  auto starting_point = toStateVector(OrbitalElements(ecc, a, 0,0,0,0, earth_mu), 0);

  double epoch = 0.0;

  auto traj = getTrajectories(earth, starting_point, solar, epoch, 86400*10); 

  for (auto t : traj) {
    string endstr[] = {"timedout", "crash", "soi_leave", "soi_enter"};
    cout << t.body << " " << t.orbit.e() << " " << t.end_time << " " << endstr[t.end-timedout] << endl;
  }
}

void testStateVector() {
  double earth_r = 6371;
  double earth_mu = 3.986004418E5;

  double ecc = 10.0;
  double peri = earth_r+180;
  double a = (ecc==1)?peri:(peri/(1-ecc));
  auto oe = OrbitalElements(ecc, a, 0,0,0,0, earth_mu);

  double epoch = 0.0;

  while (epoch < 10000000) {
    auto rv = toStateVector(oe, epoch);
    cout << epoch << ": " << glm::to_string(rv.r()) << " " << glm::to_string(rv.v()) << endl;
    epoch += 10+0.1*epoch;
  }
}

void testWindow() {

  double earth_r = 6371;
  double earth_mu = 3.986004418E5;

  double ecc = 1.4;
  double peri = earth_r*2;
  double a = (ecc==1)?peri:(peri/(1-ecc));
  auto oe = OrbitalElements(ecc, a, 0,0,0,-10, earth_mu);


  Display display;
  display.createWindow();
  display.setScale(earth_r*50);

  double epoch = 0.0;

  while (display.isOpen()) {
    display.manageEvents();
    display.clear();
    display.addPlanet(glm::dvec3(0.0), earth_r);
    display.addTrajectory(glm::dvec3(0.0), oe, std::max(0.0, epoch-1000), epoch, 100);

    display.draw();
    epoch += 200;
  }

}

int main(void) {
  testWindow();
}