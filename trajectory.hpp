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
  double En, cosTrueAnomaly, sinTrueAnomaly;
  if (isParabola) {
    En = meanToEccParabola(meanAnomaly);
    const double d = En*En+1;
    cosTrueAnomaly = (1-En*En)/d;
    sinTrueAnomaly = (2*En)/d;
  } else if (isHyperbola) {
    En = meanToEccHyperbola(meanAnomaly, e);
    const double d = 1 - e*cosh(En);
    cosTrueAnomaly = (cosh(En)-e)/d;
    sinTrueAnomaly = (sqrt(e*e-1)*sinh(En))/d;
  } else if (isEllipse) {
    En = meanToEccEllipse(meanAnomaly, e);
    const double d = 1 - e*cos(En);
    cosTrueAnomaly = (cos(En)-e)/d;
    sinTrueAnomaly = (sqrt(1-e*e)*sin(En))/d;
  }
  // Distance from parent body
  double dist;
  if (isParabola) dist = (2*a)/(1+cosTrueAnomaly);
  else dist = a*((1-e*e)/(1+e*cosTrueAnomaly));
  // Position of body
  const dvec3 posInPlane = dist*dvec3(
    cosTrueAnomaly,
    sinTrueAnomaly,
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
OrbitalElements toOrbitalElements(const StateVector sv, double mu) {

}