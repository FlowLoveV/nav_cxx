#pragma once

#include "sensors/gnss/observation.hpp"
#include "utils/space.hpp"

namespace navp::sensors::gnss {

// forward declaration
class EphemerisSolver;
class EphemerisResult;
class TropModel;
class IonoModel;

class GnssCorrectionsHandler;
struct GnssCorrections;

#pragma pack(push, 1)
struct NAVP_EXPORT GnssCorrections {
  Sv sv;
  f64 trop_corr, iono_corr;
};
#pragma pack(pop)

class NAVP_EXPORT GnssCorrectionsHandler {
 public:
  GnssCorrectionsHandler& set_time(EpochUtc time) noexcept;
  GnssCorrectionsHandler& set_trop_model(TropModel* trop_model) noexcept;
  GnssCorrectionsHandler& set_iono_model(IonoModel* iono_model) noexcept;
  GnssCorrectionsHandler& set_station_position(const utils::CoordinateBlh& coord) noexcept;
  GnssCorrectionsHandler& target_obs(const GnssObsRecord& obs_record) noexcept;
  GnssCorrectionsHandler& target_eph_solver(const EphemerisSolver& eph_solver) noexcept;

  auto handle(Sv sv) -> GnssCorrections;
  auto handle(const std::vector<Sv>& sv) -> std::vector<GnssCorrections>;

 protected:
  EpochUtc time_{};
  const utils::CoordinateBlh* station_pos_ = nullptr;
  const std::map<Sv, std::shared_ptr<GObs>>* obs_map_ = nullptr;
  const std::map<Sv, EphemerisResult>* eph_result_ = nullptr;
  TropModel* trop_model_ = nullptr;
  IonoModel* iono_model_ = nullptr;
};

}  // namespace navp::sensors::gnss