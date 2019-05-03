#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <optional>
#include <limits>

#include "trajectory.hpp"

class Body {
	double mu;
  double radius;
  double rSOI = std::numeric_limits<double>::infinity();
  std::optional<OrbitalElements> orbit;
public:
  Body(double mu, double radius) { 
    this->mu = mu; this->radius = radius; orbit = {};
  }
  Body(double mu, double radius, OrbitalElements orbit) { 
    this->mu = mu; this->radius = radius, this->orbit = orbit;
  }
  void setRSOI(double parent_mu) {
    if (!orbit) return;
    rSOI = this->orbit->a()*pow(this->mu/parent_mu, 0.4);
  }
  double getMu() const { return mu; }
  double getRSOI() const { return rSOI; }
  double getRadius() const { return radius; }
  OrbitalElements getOrbit() const { return orbit.value(); }
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

  const Body &getBody(int id) const {
    return bodies.at(id);
  }

  int getParent(int id) const {
    return parents.at(id);
  }

  std::vector<int> getChildren(int id) const {
    std::vector<int> children;
    for (size_t i=0;i<parents.size();i++) {
      if (parents[i] == id) children.push_back(i);
    }
    return children;
  }

};