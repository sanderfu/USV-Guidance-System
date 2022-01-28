//Example from: https://stackoverflow.com/questions/34767736/why-is-boostgeometry-geographic-vincenty-distance-inaccurate-around-the-equato

/// GeographicLib  WGS84 distance

// Note: M_PI is not part of the C or C++ standards, _USE_MATH_DEFINES enables it
#define _USE_MATH_DEFINES
#include <GeographicLib/Geodesic.hpp>
#include <cmath>
#include <iostream>
#include <ios>

// WGS 84 parameters from: Eurocontrol WGS 84 Implementation Manual
// Version 2.4 Chapter 3, page 14

/// The Semimajor axis measured in metres.
/// This is the radius at the equator.
constexpr double a = 6378137.0;

/// Flattening, a ratio.
/// This is the flattening of the ellipse at the poles
constexpr double f = 1.0/298.257223563;

/// The Semiminor axis measured in metres.
/// This is the radius at the poles.
/// Note: this is derived from the Semimajor axis and the flattening.
/// See WGS 84 Implementation Manual equation B-2, page 69.
constexpr double b = a * (1.0 - f);

int main(int /*argc*/, char ** /*argv*/)
{
  const GeographicLib::Geodesic& geod(GeographicLib::Geodesic::WGS84());

  std::cout.setf(std::ios::fixed);

  std::cout << "WGS 84 values (metres)\n";
  std::cout << "\tSemimajor axis:\t\t"   << a << "\n";
  std::cout << "\tFlattening:\t\t"       << f << "\n";
  std::cout << "\tSemiminor axis:\t\t"   << b << "\n\n";

  std::cout << "\tSemimajor distance:\t" << M_PI * a << "\n";
  std::cout << "\tSemiminor distance:\t" << M_PI * b << "\n";
  std::cout << std::endl;

  // Min value for delta. 0.000000014 causes boost Andoyer to fail.
  const double DELTA(0.000000015);

  std::pair<double, double> near_equator_east (0.0, 90.0 - DELTA);
  std::pair<double, double> near_equator_west (0.0, -90.0 + DELTA);

  std::cout << "GeographicLib near antipodal\n";
  double distance_metres(0.0);
  geod.Inverse(near_equator_east.first, near_equator_east.second,
               near_equator_west.first, near_equator_west.second, distance_metres);
  std::cout << "\tSemimajor distance:\t" << distance_metres << "\n";

  std::pair<double, double> near_north_pole   (90.0 - DELTA, 0.0);
  std::pair<double, double> near_south_pole   (-90.0 + DELTA, 0.0);

  geod.Inverse(near_north_pole.first, near_north_pole.second,
               near_south_pole.first, near_south_pole.second, distance_metres);
  std::cout << "\tSemiminor distance:\t" << distance_metres << "\n\n";

  std::pair<double, double> equator_east (0.0, 90.0);
  std::pair<double, double> equator_west (0.0, -90.0);

  std::cout << "GeographicLib antipodal\n";
  geod.Inverse(equator_east.first, equator_east.second,
               equator_west.first, equator_west.second, distance_metres);
  std::cout << "\tSemimajor distance:\t" << distance_metres << "\n";

  std::pair<double, double> north_pole   (90.0, 0.0);
  std::pair<double, double> south_pole   (-90.0, 0.0);

  geod.Inverse(north_pole.first, north_pole.second,
               south_pole.first, south_pole.second, distance_metres);
  std::cout << "\tSemiminor distance:\t" << distance_metres << "\n\n";

  std::pair<double, double> JFK   (40.6, -73.8);
  std::pair<double, double> LHR   (51.6, -0.5);

  std::cout << "GeographicLib verify distance\n";
  geod.Inverse(JFK.first, JFK.second,
               LHR.first, LHR.second, distance_metres);
  std::cout << "\tJFK to LHR distance:\t" << distance_metres << std::endl;

  return 0;
}