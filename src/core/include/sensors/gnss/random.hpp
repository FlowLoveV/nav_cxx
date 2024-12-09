#pragma once

#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::sensors::gnss {

// forward declaration
class EphemerisSolver;
class EphemerisResult;

class GnssRandomHandler;

class GnssRandomHandler {
 public:
  GnssRandomHandler& set_time(EpochUtc time) noexcept;
  GnssRandomHandler& set_random_model(RandomModelEnum type) noexcept;
  GnssRandomHandler& target_obs(const GnssObsRecord& obs_record) noexcept;
  GnssRandomHandler& target_eph_solver(const EphemerisSolver& eph_solver) noexcept;

  const Sig* handle(Sv sv, ObsCodeEnum code);
  // bool handle(const std::vector<Sv>& sv);

 protected:
  bool handlable() const noexcept;

  EpochUtc time_{};
  std::map<Sv, std::shared_ptr<GObs>>* obs_map_ = nullptr;
  const std::map<Sv, EphemerisResult>* eph_result_ = nullptr;
  RandomModelEnum type_;
};
}  // namespace navp::sensors::gnss