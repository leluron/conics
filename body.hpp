#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <optional>
#include <limits>

#include "trajectory.hpp"

class Body {
	double mu;
  double rSOI = std::numeric_limits<double>::infinity();
  std::optional<OrbitalElements> orbit;
public:
  Body(double mu) { this->mu = mu; orbit = {};}
  Body(double mu, OrbitalElements orbit) { this->mu = mu; this->orbit = orbit;}
  void setRSOI(double parent_mu) {
    if (!orbit) return;
    rSOI = this->orbit->a()*pow(this->mu/parent_mu, 0.4);
  }
  double getMu() { return mu; }
  double getRSOI() { return rSOI; }
  OrbitalElements getOrbit() { return orbit.value(); }
};

class System {
  std::vector<Body> bodies;
  std::vector<int> parents;
public:
  int init(Body b) {
    bodies.clear();
    parents.clear();
    bodies.push_back(b),
    parents.push_back(-1);
    return 0;
  }

  int add(Body b, int parent) {
    int id = bodies.size();
    if (parent >= id) return -1;
    parents.push_back(parent);
    bodies.push_back(b);
    bodies.back().setRSOI(getBody(parent).getMu());
    return id;
  }

  Body &getBody(int id) {
    return bodies.at(id);
  }

  int getParent(int id) {
    return parents.at(id);
  }
};