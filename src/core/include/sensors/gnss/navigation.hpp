#pragma once

#include "space_time/time.hpp"
#include "types.hpp"

namespace nav::sensor::gnss {

class RecordGnssNav;

/// `gnss navigation record`
///
/// implment for GPS/BDS/GALIEO/QZSS
class RecordGnssNav {
 public:
  struct KeplerOrbit {
    /// `a` semi major axis (m)
    /// `e` Eccentricity (n.a)
    /// `i_0` Inclination angle at reference time (semicircles)
    /// `omega_0` Longitude of ascending node at reference time (semicircles)
    /// `m_0` Mean anomaly at reference time (semicircles)
    /// `omega` argument of perigee (semicircles)
    /// `toe` time of issue of ephemeris
    f64 a, e, i_0, omega_0, m_0, omega, toe;
  };
  struct Clock {
    f64 a0, a1, a2;
  };
  struct KeplerOrbitPerturbations {
    // `dn` Mean motion difference from computed value [semicircles.s-1]
    // `i_dot` Inclination rate of change [semicircles.s-1]
    // `omega_dot` Right ascension rate of change [semicircles.s^-1]
    // `cus` Amplitude of sine harmonic correction term of the argument
    // `cuc` Amplitude of cosine harmonic correction term of the argument
    // `cis` Amplitude of sine harmonic correction term of the angle of inclination [rad]
    // `cic` Amplitude of cosine harmonic correction term of the angle of inclination [rad]
    // `crs` Amplitude of sine harmonic correction term of the orbit radius [m]
    // `crc` Amplitude of cosine harmonic correction term of the orbit radius [m]
    f64 dn, i_dot, omega_dot, cus, cuc, cis, cic, crs, crc;
    // `a_dot` specially for CNAV
    std::optional<f64> a_dot;
  };
  struct Ephemeris {
    KeplerOrbit orbit;
    Clock clock;
    KeplerOrbitPerturbations perturbations;
  };

  typedef std::map<Sv, std::map<Epoch<UTC>, Ephemeris>> RecordType;

 private:
  RecordType record_;
};

}  // namespace nav::sensor::gnss