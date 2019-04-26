#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>

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

static double meanToEccParabola(const double mean, const double q) {
  // Newton to find eccentric anomaly (En)
  double En = mean; // Starting value of En
  const int it = 20; // Number of iterations
  for (int i=0;i<it;++i)
    En -= (q*En+En*En*En/6-mean)/(q+En*En/2);
  return En;
}

class StateVector {
  glm::dvec3 _r, _v;
public:
  StateVector(glm::dvec3 r, glm::dvec3 v) { _r = r; _v = v;}
  glm::dvec3 r() const { return _r; }
  glm::dvec3 v() const { return _v; }
};

class OrbitalElements {
  // for parabolic orbits a is q
  double _e, _a, _i, _an, _arg, _m0;
public:
  OrbitalElements(double e, double a, double i, double an, double arg, double m0) {
    _e = e;
    _a = a;
    _i = i;
    _an = an;
    _arg = arg;
    _m0 = m0;
  }
  double e() const { return _e; }
  double a() const { return _a; }
  double i() const { return _i; }
  double an() const { return _an; }
  double arg() const { return _arg; }
  double m0() const { return _m0; }
};

using namespace glm;

StateVector toStateVector(const OrbitalElements oe, double mu, double epoch) {
  const double a = oe.a();
  const double e = oe.e();
  // Conics
  const bool isParabola = (e==1);
  const bool isHyperbola = (e>1);
  const bool isEllipse = (e<1);
  // Mean Anomaly compute
  const double meanMotion = sqrt(mu/(isParabola?1:abs(a*a*a)));
  double meanAnomaly = epoch*meanMotion + oe.m0();
  // Cap true anomaly in case of elliptic orbit
  if (isEllipse)
    meanAnomaly = fmod(meanAnomaly, 2*pi<float>());
  // Mean anomaly to Eccentric anomaly, to true anomaly
  double En, cosTrueAnomaly, sinTrueAnomaly;
  if (isParabola) {
    En = meanToEccParabola(meanAnomaly, a);
    const double d = 2*a+En*En;
    cosTrueAnomaly = (2*a-En*En)/d;
    sinTrueAnomaly = (2*sqrt(2*a)*En)/d;
  } else if (isHyperbola) {
    En = meanToEccHyperbola(meanAnomaly, e);
    const double d = 1 - e*cosh(En);
    cosTrueAnomaly = (cosh(En)-e)/d;
    sinTrueAnomaly = (sqrt(e*e-1)*sinh(En))/-d;
  } else if (isEllipse) {
    En = meanToEccEllipse(meanAnomaly, e);
    const double d = 1 - e*cos(En);
    cosTrueAnomaly = (cos(En)-e)/d;
    sinTrueAnomaly = (sqrt(1-e*e)*sin(En))/d;
  }
  // Distance from parent body
  double dist;
  if (isParabola) dist = a + (En*En/2);
  else dist = a*((1-e*e)/(1+e*cosTrueAnomaly));
  // Position of body
  const dvec3 posInPlane = dist*dvec3(
    cosTrueAnomaly,
    sinTrueAnomaly,
    0.0);

  // Velocity of body
  const double v = (isParabola)?sqrt(mu/dist):sqrt(mu*abs(a))/dist;
  dvec3 velInPlane;
  if (isParabola) velInPlane = dvec3(
    -sqrt(1-cosTrueAnomaly),
    sqrt(cosTrueAnomaly+1),
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
      rotate(dquat(1,0,0,0), oe.an() , dvec3(0,0,1))
    * rotate(dquat(1,0,0,0), oe.i()  , dvec3(0,1,0))
    * rotate(dquat(1,0,0,0), oe.arg(), dvec3(0,0,1));
  return {q*posInPlane, q*(v*velInPlane)};
}
OrbitalElements toOrbitalElements(const StateVector sv, double mu, double epoch) {
  const dvec3 rv = sv.r();
  const dvec3 v = sv.v();
  const double r = length(rv);

  // Eccentricity
  const dvec3 h = cross(rv, v);
  const dvec3 dir = normalize(rv);
  const dvec3 ev = cross(v, h)/mu - dir;
  double e = length(ev);
  const dvec3 edir = normalize(ev);
  dvec3 nv = cross(dvec3(0,0,1), h);
  if (length(nv) == 0) nv = dvec3(1,0,0);
  else nv = normalize(nv);

  // Semi-major axis (or q for parabolic orbits)
  const double a_inv = (2.0/r - length2(v)/mu);
  double a;
  if (e==1) a = length2(h)/(2*mu);
  else a = 1.0/a_inv;

  // True anomaly
  double trueAnomaly = acos(dot(edir,dir));
  if (dot(rv, v) < 0) trueAnomaly = 2*pi<double>() - trueAnomaly;

  // Eccentric anomaly
  double En, meanAnomaly;
  if (e == 1) {
    En = tan(trueAnomaly/2);
    meanAnomaly = En + En*En*En/3;
  } else if (e < 1) {
    En = 2*atan2(tan(trueAnomaly/2), sqrt((1+e)/(1-e)));
    meanAnomaly = En - e*sin(En);
  } else {
    En = 2*atanh(sqrt((e-1)/(1+e))*tan(trueAnomaly/2));
    meanAnomaly = e*sinh(En) - En;
  }
  // Mean anomaly
  const double meanMotion = sqrt(mu/abs(a*a*a));
  const double m0 = meanAnomaly - epoch*meanMotion;

  // Rotation
  const double i = acos(h.z/length(h));
  double an = acos(nv.x);
  if (nv.y < 0) an = 2.0*pi<double>() - an;
  double arg = acos(dot(nv,edir));
  if (ev.z < 0 ) arg = 2.0*pi<double>() - arg;

  return OrbitalElements(e, a, i, an, arg, m0);
}