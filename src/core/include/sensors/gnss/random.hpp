#pragma once

#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::sensors::gnss {

// forward declaration
class EphemerisSolver;
class EphemerisResult;

class GnssRandomHandler;

class GnssRandomHandler {
 public:
  GnssRandomHandler& set_obs(const GObs* obs) noexcept;
  GnssRandomHandler& set_sv_info(const EphemerisResult* eph_result) noexcept;

  const Sig* handle(ObsCodeEnum code, RandomModelEnum model) const noexcept;

 protected:
  GObs* obs_ = nullptr;
  const EphemerisResult* sv_info_ = nullptr;
};
}  // namespace navp::sensors::gnss