#pragma once

#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/eigen.hpp"
#include "utils/macro.hpp"
#include "utils/space.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

using utils::NavVector3f64;

class AtmosphereHandler;
class TropHandler;
class IonoHandler;

class NAVP_EXPORT AtmosphereHandler {
 public:
  AtmosphereHandler& set_time(const EpochUtc& tr) noexcept;

  AtmosphereHandler& set_sv_info(const EphemerisResult* eph_result) noexcept;

  AtmosphereHandler& set_station_pos(const utils::CoordinateBlh* pos) noexcept;

 protected:
  bool solvable() const noexcept;

  const utils::CoordinateBlh* station_pos_{};
  const EpochUtc* tr_;
  const EphemerisResult* sv_info_{};
};

class NAVP_EXPORT TropHandler : public AtmosphereHandler {
 public:
  f64 handle(TropModelEnum model) const noexcept;
};

class NAVP_EXPORT IonoHandler : public AtmosphereHandler {
 public:
  f64 handle(IonoModelEnum model) const noexcept;
};

}  // namespace navp::sensors::gnss