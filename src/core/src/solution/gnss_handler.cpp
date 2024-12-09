#include "solution/gnss_handler.hpp"

#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/corrections.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "sensors/gnss/random.hpp"

namespace navp::solution {

using navp::sensors::gnss::GnssCorrectionsHandler;
using navp::sensors::gnss::GnssRandomHandler;

void GnssHandler::initialize(const NavConfigManger& config, bool is_rover) {
  initialize_navigation(config, is_rover);
  initialize_observation(config, is_rover);
  initialize_atmosphere_model(config);
  initialize_obs_code(config);
  initialize_random_model(config);
  initialize_solution_mode(config);
}

void GnssHandler::initialize_navigation(const NavConfigManger& config, bool is_rover) {
  // initialize ephemeris
  auto rover_nav = is_rover ? config.rover_nav() : config.base_nav();
  if (rover_nav.is_err()) [[unlikely]] {
    throw rover_nav.unwrap_err_unchecked();
  } else {
    nav_ = std::move(rover_nav.unwrap_unchecked());
  }
  // initialize ephemeris solver
  eph_solver_ = std::make_unique<EphemerisSolver>();
  std::ranges::for_each(nav_, [&](const GnssNavRecord& record) { eph_solver_->add_ephemeris(record.nav.get()); });
}

void GnssHandler::initialize_observation(const NavConfigManger& config, bool is_rover) {
  // initialize observation stream
  auto rover_obs_stream = is_rover ? config.rover_obs() : config.base_obs();
  if (rover_obs_stream.is_err()) [[unlikely]] {
    throw rover_obs_stream.unwrap_err_unchecked();
  } else {
    obs_stream_ = std::move(rover_obs_stream.unwrap_unchecked());
  }
  // initialize observation
  obs_ = std::make_unique<GnssObsRecord>();
}

void GnssHandler::initialize_atmosphere_model(const NavConfigManger& config) {
  // initialize trop model
  auto trop = config.trop_model();
  if (trop.is_err()) [[unlikely]] {
    throw trop.unwrap_err_unchecked();
  } else {
    trop_model_ = std::make_unique<TropModel>();
    trop_model_->set_model(trop.unwrap_unchecked());
  }
  // initialize iono model
  auto iono = config.iono_model();
  if (iono.is_err()) [[unlikely]] {
    throw iono.unwrap_err_unchecked();
  } else {
    iono_model_ = std::make_unique<IonoModel>();
    iono_model_->set_model(iono.unwrap_unchecked());
  }
}

void GnssHandler::initialize_obs_code(const NavConfigManger& config) {
  auto obs_code = config.enabled_code();
  if (obs_code.is_err()) [[unlikely]] {
    throw obs_code.unwrap_err_unchecked();
  } else {
    obs_code_ = std::move(obs_code.unwrap_unchecked());
  }
}

void GnssHandler::initialize_random_model(const NavConfigManger& config) {
  auto random_model = config.random_model();
  if (random_model.is_err()) [[unlikely]] {
    throw random_model.unwrap_err_unchecked();
  } else {
    random_model_ = random_model.unwrap_unchecked();
  }
}

void GnssHandler::initialize_solution_mode(const NavConfigManger& config) {
  auto solution_mode = config.solution_mode();
  if (solution_mode.is_err()) {
    throw solution_mode.unwrap_err_unchecked();
  } else {
    solution_mode_ = solution_mode.unwrap_unchecked();
  }
}

EphemerisSolver& GnssHandler::ephemeris_solver() noexcept { return *eph_solver_; }

GnssObsRecord& GnssHandler::observation() noexcept { return *obs_; }

TropModel& GnssHandler::trop_model() noexcept { return *trop_model_; }

bool GnssHandler::get_next_observation() noexcept {
  if (!obs_stream_->eof()) [[likely]] {
    obs_->get_record(*obs_stream_);
    return true;
  }
  return false;
}

auto GnssHandler::current_available_satellites() noexcept -> std::vector<Sv> {
  auto& [time, obs] = obs_->latest();
  return eph_solver_->solve_sv_status(time, obs);
}

auto GnssHandler::generate_corrections_handler(EpochUtc time, const utils::CoordinateXyz& station_pos) noexcept
    -> std::function<GnssCorrections(Sv)> {
  return [handler = GnssCorrectionsHandler{}
                        .set_time(time)
                        .target_obs(*obs_)
                        .target_eph_solver(*eph_solver_)
                        .set_trop_model(trop_model_.get())
                        .set_iono_model(iono_model_.get())
                        .set_station_position(station_pos)](Sv sv) mutable { return handler.handle(sv); };
}

auto GnssHandler::generate_random_handler(EpochUtc time) noexcept -> std::function<const Sig*(Sv, ObsCodeEnum)> {
  return [handler = GnssRandomHandler{}
                        .set_time(time)
                        .target_obs(*obs_)
                        .target_eph_solver(*eph_solver_)
                        .set_random_model(random_model_)](Sv sv, ObsCodeEnum code) mutable {
    return handler.handle(sv, code);
  };
}

}  // namespace navp::solution