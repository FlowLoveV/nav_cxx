#pragma once

#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::sensors::gnss {

// forward declaration
class EphemerisResult;

class GnssRandomHandler;

class GnssRandomHandler {
 public:
  enum EvaluateRandomOptions : u8 {
    Pseudorange = 0b01,
    Carrier = 0b10,
    Both = 0b11,
  };

  GnssRandomHandler& set_model(RandomModelEnum model) noexcept;
  GnssRandomHandler& set_options(EvaluateRandomOptions options) noexcept;
  GnssRandomHandler& set_sv_info(const EphemerisResult* eph_result) noexcept;

  const Sig* handle(const Sig* sig) const noexcept;

 protected:
  EvaluateRandomOptions options_ = EvaluateRandomOptions::Both;
  RandomModelEnum model_ = RandomModelEnum::STANDARD;
  const EphemerisResult* sv_info_ = nullptr;
};
}  // namespace navp::sensors::gnss