#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

static double meanToEccEllipse(const double mean, const double ecc)
{
  // Newton to find eccentric anomaly (En)
  double En = mean; // Starting value of En
  const int it = 20; // Number of iterations
  for (int i=0;i<it;++i)
    En -= (En - ecc*sin(En)-mean)/(1-ecc*cos(En));
  return En;
}

static double meanToEccHyperbola(const double mean, const double ecc)
{
  // Newton to find eccentric anomaly (En)
  double En = mean; // Starting value of En
  const int it = 20; // Number of iterations
  for (int i=0;i<it;++i)
    En -= (ecc*sinh(En)-En-mean)/(ecc*cosh(En)-1);
  return En;
}

class StateVector {
public:
  glm::dvec3 p, v;
};

class OrbitalElements {
public:
  double e, a, i, an, arg, m0;
};

using namespace glm;

StateVector toStateVector(const OrbitalElements oe, double mu, double epoch) {
  // Mean Anomaly compute
  const double meanMotion = sqrt(mu/abs(oe.a*oe.a*oe.a));
  double meanAnomaly = epoch*meanMotion + oe.m0;
  if (oe.e < 1)
    meanAnomaly = fmod(meanAnomaly, 2*pi<float>());
  // Mean anomaly to Eccentric
  const double En = ((oe.e>=1)?meanToEccHyperbola:meanToEccEllipse)(meanAnomaly, oe.e);
  // Eccentric anomaly to True anomaly
  const double trueAnomaly = 
    (oe.e>=1)
    ?2*atan2(sqrt(1+oe.e)*sinh(En/2), sqrt(oe.e-1)*cosh(En/2))
    :2*atan2(sqrt(1+oe.e)*sin(En/2), sqrt(1-oe.e)*cos(En/2));
  // Distance from parent body
  const double dist = oe.a*((1-oe.e*oe.e)/(1+oe.e*cos(trueAnomaly)));
  // Plane changes
  const dvec3 posInPlane = dist*dvec3(
    cos(trueAnomaly),
    sin(trueAnomaly),
    0.0);
  const double v = sqrt(mu*oe.a)/dist;
  const dvec3 velInPlane = v*dvec3(
    -sin(En),
    sqrt(1-oe.e*oe.e)*cos(En),
    0.0);
  const dquat q =
      rotate(dquat(1,0,0,0), oe.an , dvec3(0,0,1))
    * rotate(dquat(1,0,0,0), oe.i  , dvec3(0,1,0))
    * rotate(dquat(1,0,0,0), oe.arg, dvec3(0,0,1));
  return {q*posInPlane, q*velInPlane};
}