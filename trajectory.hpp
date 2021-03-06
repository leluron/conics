#pragma once

#include <tuple>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <functional>

bool isNan(double x) {return x!=x;}

double newton_raphson(std::function<double(double)> f, std::function<double(double)> df, double x0) {
  double x = x0;
  while (isNan(f(x)/df(x))) x *= 0.5;

  int max_iterations = 1E3;
  for (int i=0;i<max_iterations;i++) {
    double diff = f(x)/df(x);
    if (diff == 0.0) break;
    x -= diff;
  }
  return x;
}
// Mean anomaly to eccentric anomaly
// e is eccentricity
// q is periapsis radius in case of parabolic trajectory
double meanToEcc(double mean, double e) {
  // Newton to find eccentric anomaly (En)
  if (e==1) return newton_raphson([mean](double En){return En+En*En*En/3-mean;},[](double En){return 1+En*En;}, mean);
  else if (e<1) return newton_raphson([mean, e](double En){return En-e*sin(En)-mean;}, [e](double En){return 1-e*cos(En);}, mean);
  else return newton_raphson([mean,e](double En){return e*sinh(En)-En-mean;}, [e](double En){return e*cosh(En)-1;}, mean);
}


// True anomaly to eccentric anomaly
// e is eccentricity
double trueToEcc(double trueAnomaly, double e) {
  if (e == 1) return tan(trueAnomaly/2);
  else if (e < 1) return 2*atan2(tan(trueAnomaly/2), sqrt((1+e)/(1-e)));
  else return 2*atanh(sqrt((e-1)/(1+e))*tan(trueAnomaly/2));
}

// Eccentric anomaly to Mean anomaly
// e is eccentricity
double eccToMean(double En, double e) {
  if (e == 1) return En + En*En*En/3;
  else if (e < 1) return En - e*sin(En);
  else return e*sinh(En) - En;
}

// Eccentric anomaly to true anomaly
// e is eccentricity
// a is semi major axis
// Returns cos and sin of true anomaly
std::tuple<double, double> eccToTrue(double En, double e) {
  if (e == 1) {
    double d = 1+En*En;
    return std::make_tuple((1-En*En)/d,(2*En)/d);
  } else if (e < 1) {
    double d = 1 - e*cos(En);
    return std::make_tuple((cos(En)-e)/d, (sqrt(1-e*e)*sin(En))/d);
  } else {
    double d = 1 - e*cosh(En);
    return std::make_tuple((cosh(En)-e)/d, (sqrt(e*e-1)*sinh(En))/-d);
  }
}

double getMeanMotion(double mu, double e, double a) {
  if (e==1) return sqrt(mu/(2*a*a*a));
  else if (e<1) return sqrt(mu/(a*a*a));
  else return sqrt(-mu/(a*a*a));
}

// Position and velocity around body center
class StateVector {
  glm::dvec3 _r, _v;
public:
  StateVector(glm::dvec3 r, glm::dvec3 v) { _r = r; _v = v;}
  glm::dvec3 r() const { return _r; }
  glm::dvec3 v() const { return _v; }
};

StateVector operator+(const StateVector sv1, const StateVector sv2) {
  return StateVector(sv1.r() + sv2.r(), sv1.v() + sv2.v());
}

// Kepler elements
class OrbitalElements {
  double _e, _a, _i, _an, _arg, _m0, _n;
public:
  OrbitalElements(double e, double a, double i, double an, double arg, double m0, double mu) {
    _e = e;
    _a = a;
    _i = i;
    _an = an;
    _arg = arg;
    _m0 = m0;
    _n = getMeanMotion(mu, e, a);
  }
  double e() const { return _e; } // eccentricity
  double a() const { return _a; } // semi major axis; altitude of periapsis for parabolic
  double i() const { return _i; } // inclination (radians)
  double an() const { return _an; } // longitude of ascending node (radians)
  double arg() const { return _arg; } // argument of periapsis (radians)
  double m0() const { return _m0; } // mean anomaly at epoch = 0
  double n() const { return _n; } // mean motion
};

using namespace glm;

#include <iostream>

// Orbital elements to state vectors
// mu is gravitational parameter of body
// epoch is seconds elapsed since starting epoch
StateVector toStateVector(OrbitalElements oe, double epoch) {
  double a = oe.a();
  double e = oe.e();
  // Mean Anomaly compute
  double meanAnomaly = epoch*oe.n() + oe.m0();
  // Cap true anomaly in case of elliptic orbit
  if (e<1) meanAnomaly = fmod(meanAnomaly, 2*pi<float>());
  // Mean anomaly to Eccentric anomaly, to true anomaly
  double En = meanToEcc(meanAnomaly, e);
  double cosTrueAnomaly, sinTrueAnomaly;
  std::tie(cosTrueAnomaly, sinTrueAnomaly) = eccToTrue(En, e);
  // Distance from parent body
  double p = a*((e==1)?2:(1-e*e));
  double dist = p/(1+e*cosTrueAnomaly);
  // Position of body
  dvec3 posInPlane = dist*dvec3(
    cosTrueAnomaly,
    sinTrueAnomaly,
    0.0);

  // Velocity of body (TODO)
  dvec3 velInPlane = dvec3(0);
  /*
  double v = (e==1)?sqrt(mu/dist):sqrt(mu*abs(a))/dist;
  if (e==1) velInPlane = dvec3(
    -sqrt(1-cosTrueAnomaly),
    sqrt(cosTrueAnomaly+1),
    0.0);
  else if (e<1) velInPlane = dvec3(
    -sin(En),
    cos(En)*sqrt(1-e*e),
    0.0);
  else velInPlane = dvec3(
    -sinh(En),
    cosh(En)*sqrt(e*e-1),
    0.0);
  */

  // Rotation
  dquat q =
      rotate(dquat(1,0,0,0), oe.an() , dvec3(0,0,1))
    * rotate(dquat(1,0,0,0), oe.i()  , dvec3(0,1,0))
    * rotate(dquat(1,0,0,0), oe.arg(), dvec3(0,0,1));
  return {q*posInPlane, q*velInPlane};
}

// returns epoch of next crossing of given altitude
// +inf for never
double epochReachAltitude(OrbitalElements oe, double alt, double epoch) {
  double inf = std::numeric_limits<double>::infinity();
  double a = oe.a();
  double e = oe.e();

  double meanMotion = oe.n();

  if (e==1) {
    // parabola :
    // never if q above altitude
    if (alt < a) return inf;
    // get mean anomaly
    double trueAnomaly = acos((2*a/alt)-1);
    double En = trueToEcc(trueAnomaly, e);
    double meanAnomaly = eccToMean(En, e);
    // get next epoch
    double epoch0 = (meanAnomaly - oe.m0())/meanMotion;
    double epoch1 = (-meanAnomaly - oe.m0())/meanMotion;
    if (epoch < epoch0) return epoch0;
    else if (epoch < epoch1) return epoch1;
    else return inf;
  } else if (e<1) {
    // ellipse : 
    // never if alt out of bounds of min and max alt
    double min = a*((1-e*e)/(1+e));
    double max = a*((1-e*e)/(1-e));
    if (alt < min || alt > max) return inf;
    // get mean anomaly
    double trueAnomaly = (e==0)?0:acos(((a*(1-e*e)/alt)-1)/e);
    double En = trueToEcc(trueAnomaly, e);
    double meanAnomaly = eccToMean(En, e);
    double currentMeanAnomaly = fmod(epoch*meanMotion + oe.m0(), 2*pi<float>());
    // get next epoch
    if (currentMeanAnomaly >= meanAnomaly && currentMeanAnomaly < (2*pi<float>() - meanAnomaly)) meanAnomaly = 2*pi<float>() - meanAnomaly;
    double diff = fmod(meanAnomaly - currentMeanAnomaly + 2*pi<float>(), 2*pi<float>());
    return epoch + diff/meanMotion;
  } else {
    // never if periapsis above altitude
    if (alt < (a*(1-e*e)/(1+e))) return inf;
    // get mean anomaly
    double trueAnomaly = acos(((a*(1-e*e)/alt)-1)/e);
    double En = trueToEcc(trueAnomaly, e);
    double meanAnomaly = eccToMean(En, e);
    // get next epoch
    double epoch0 = (meanAnomaly - oe.m0())/meanMotion;
    double epoch1 = (-meanAnomaly - oe.m0())/meanMotion;
    if (epoch < epoch0) return epoch0;
    else if (epoch < epoch1) return epoch1;
    else return inf;
  }
}

OrbitalElements toOrbitalElements(StateVector sv, double mu, double epoch) {
  dvec3 rv = sv.r();
  dvec3 v = sv.v();
  double r = length(rv);

  // Eccentricity
  dvec3 h = cross(rv, v);
  dvec3 dir = normalize(rv);
  dvec3 ev = cross(v, h)/mu - dir;
  double e = length(ev);
  dvec3 edir = normalize(ev);
  dvec3 nv = cross(dvec3(0,0,1), h);
  if (length(nv) == 0) nv = dvec3(1,0,0);
  else nv = normalize(nv);

  // Semi-major axis (or q for parabolic orbits)
  double a = 1.0/(2.0/r - length2(v)/mu);
  if (e==1 || a == std::numeric_limits<double>::infinity()) {
    e = 1;
    a = length2(h)/(2*mu);
  }

  // True anomaly
  double trueAnomaly = acos(dot(edir,dir));
  if (dot(rv, v) < 0) trueAnomaly = 2*pi<double>() - trueAnomaly;

  // Eccentric anomaly
  double En = trueToEcc(trueAnomaly, e);
  double meanAnomaly = eccToMean(En, e);
  // Mean anomaly
  double meanMotion = getMeanMotion(mu, e, a);
  double m0 = meanAnomaly - epoch*meanMotion;

  // Rotation
  double i = acos(h.z/length(h));
  double an = acos(nv.x);
  if (nv.y < 0) an = 2.0*pi<double>() - an;
  double arg = acos(dot(nv,edir));
  if (ev.z < 0 ) arg = 2.0*pi<double>() - arg;

  return OrbitalElements(e, a, i, an, arg, m0, mu);
}