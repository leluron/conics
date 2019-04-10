#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

static double meanToEccEllipse(const double mean, const double ecc) {
  // Newton to find eccentric anomaly (En)
  double En = mean; // Starting value of En
  const int it = 20; // Number of iterations
  for (int i=0;i<it;++i)
    En -= (En - ecc*sin(En)-mean)/(1-ecc*cos(En));
  return En;
}

static double meanToEccHyperbola(const double mean, const double ecc) {
  // Newton to find eccentric anomaly (En)
  double En = mean; // Starting value of En
  const int it = 20; // Number of iterations
  for (int i=0;i<it;++i)
    En -= (ecc*sinh(En)-En-mean)/(ecc*cosh(En)-1);
  return En;
}

static double meanToEccParabola(const double mean) {
  // Newton to find eccentric anomaly (En)
  double En = mean; // Starting value of En
  const int it = 20; // Number of iterations
  for (int i=0;i<it;++i)
    En -= (En+(En*En*En)/3-mean)/(1+En*En);
  return En;
}

class StateVector {
public:
  glm::dvec3 p, v;
};

class OrbitalElements {
public:
  double e, a, i, an, arg, m0;
  // for parabolic orbits a is q
};

using namespace glm;

StateVector toStateVector(const OrbitalElements oe, double mu, double epoch) {
  const double a = oe.a;
  const double e = oe.e;
  // Conics
  const bool isParabola = (e==1);
  const bool isHyperbola = (e>1);
  const bool isEllipse = (e<1);
  // Mean Anomaly compute
  const double meanMotion = sqrt(mu/abs(a*a*a));
  double meanAnomaly = epoch*meanMotion + oe.m0;
  // Cap true anomaly in case of elliptic orbit
  if (isEllipse)
    meanAnomaly = fmod(meanAnomaly, 2*pi<float>());
  // Mean anomaly to Eccentric anomaly, to true anomaly
  double En, trueAnomaly;
  if (isParabola) {
    En = meanToEccParabola(meanAnomaly);
    trueAnomaly = 2*atan(En);
  } else if (isHyperbola) {
    En = meanToEccHyperbola(meanAnomaly, e);
    trueAnomaly = 2*atan2(sqrt(1+e)*sinh(En/2), sqrt(e-1)*cosh(En/2));
  } else if (isEllipse) {
    En = meanToEccEllipse(meanAnomaly, e);
    trueAnomaly = 2*atan2(sqrt(1+e)*sin(En/2), sqrt(1-e)*cos(En/2));
  }
  // Distance from parent body
  double dist;
  if (isParabola) dist = (2*a)/(1+cos(trueAnomaly));
  else dist = a*((1-e*e)/(1+e*cos(trueAnomaly)));
  // Position of body
  const dvec3 posInPlane = dist*dvec3(
    cos(trueAnomaly),
    sin(trueAnomaly),
    0.0);

  // Velocity of body
  const double v = sqrt(mu*abs(a))/dist;
  dvec3 velInPlane;
  if (isParabola) velInPlane = dvec3(
    -sin(En),
    cos(En)*sqrt(2),
    0.0);
  else if (isHyperbola) velInPlane = dvec3(
    -sinh(En),
    cosh(En)*sqrt(e*e-1),
    0.0);
  else if (isEllipse) velInPlane = dvec3(
    -sin(En),
    cos(En)*sqrt(1-e*e),
    0.0);

  // Rotation
  const dquat q =
      rotate(dquat(1,0,0,0), oe.an , dvec3(0,0,1))
    * rotate(dquat(1,0,0,0), oe.i  , dvec3(0,1,0))
    * rotate(dquat(1,0,0,0), oe.arg, dvec3(0,0,1));
  return {q*posInPlane, q*(v*velInPlane)};
}