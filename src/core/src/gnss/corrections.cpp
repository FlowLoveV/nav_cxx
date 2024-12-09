#include "sensors/gnss/corrections.hpp"

#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"

namespace navp::sensors::gnss {

GnssCorrectionsHandler& GnssCorrectionsHandler::set_time(EpochUtc time) noexcept {
  time_ = time;
  return *this;
}

GnssCorrectionsHandler& GnssCorrectionsHandler::set_trop_model(TropModel* trop_model) noexcept {
  trop_model_ = trop_model;
  return *this;
}

GnssCorrectionsHandler& GnssCorrectionsHandler::set_iono_model(IonoModel* iono_model) noexcept {
  iono_model_ = iono_model;
  return *this;
}

GnssCorrectionsHandler& GnssCorrectionsHandler::set_station_position(const utils::CoordinateBlh& coord) noexcept {
  station_pos_ = &coord;
  return *this;
}

GnssCorrectionsHandler& GnssCorrectionsHandler::target_obs(const GnssObsRecord& obs_record) noexcept {
  obs_map_ = obs_record.query(time_);
  return *this;
}

GnssCorrectionsHandler& GnssCorrectionsHandler::target_eph_solver(const EphemerisSolver& brdc_eph_solver) noexcept {
  eph_result_ = brdc_eph_solver.quary_sv_status(time_);
  return *this;
}

auto GnssCorrectionsHandler::handle(Sv sv) -> GnssCorrections {
  f64 trop_coor = 0.0, iono_coor = 0.0;
  if (trop_model_) {
    trop_coor = (*trop_model_).set_time(time_).set_sv_status(eph_result_).set_station_pos(station_pos_).solve(sv);
  }
  if (iono_model_) {
    iono_coor = (*iono_model_).set_time(time_).set_sv_status(eph_result_).set_station_pos(station_pos_).solve(sv);
  }
  return GnssCorrections{.sv = sv, .trop_corr = trop_coor, .iono_corr = iono_coor};
}

auto GnssCorrectionsHandler::handle(const std::vector<Sv>& sv) -> std::vector<GnssCorrections> {
  std::vector<GnssCorrections> result(sv.size());
  for (auto it : sv) {
    result.emplace_back(handle(it));
  }
  return std::move(result);
}

}  // namespace navp::sensors::gnss