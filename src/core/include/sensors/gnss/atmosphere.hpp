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

  AtmosphereHandler& set_trop_model(TropModelEnum model) noexcept;

  AtmosphereHandler& set_iono_model(IonoModelEnum model) noexcept;

  AtmosphereHandler& set_sv_info(const EphemerisResult* eph_result) noexcept;

  f64 handle_trop(const utils::CoordinateBlh* pos) const noexcept;

  f64 handle_iono(const utils::CoordinateBlh* pos) const noexcept;

  auto sv_info() const noexcept -> const EphemerisResult*;

 protected:
  bool solvable() const noexcept;
  TropModelEnum trop_model_ = TropModelEnum::STANDARD;
  IonoModelEnum iono_model_ = IonoModelEnum::NONE;
  const EpochUtc* tr_ = nullptr;
  const EphemerisResult* sv_info_ = nullptr;
};

}  // namespace navp::sensors::gnss