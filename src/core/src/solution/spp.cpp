#include "solution/spp.hpp"

#include <ranges>

#include "sensors/gnss/constants.hpp"
#include "solution/config.hpp"

using namespace navp::sensors::gnss;

namespace navp::solution {

// this function is used to handle the position model
// - jacobian    (modeled)
// - observation (modeled)
// - weight      (modeled)
template <std::floating_point _Float_t>
void handle_spp_signal_model(algorithm::WeightedLeastSquare<_Float_t>& wls, const Sig* sig,
                             const EphemerisResult* sv_info, f64 trop_err, f64 iono_err, i32 obs_index,
                             i32 clock_index) {
  f64 distance = 0;
  auto& jacobian = wls.jacobian();
  jacobian.row(obs_index).setZero();  // reset jacobian
  auto& observation = wls.observation();
  auto& parameter = wls.parameter();
  auto& weight = wls.weight();
  auto position = utils::CoordinateXyz(wls.parameter().block(0, 0, 3, 1));
  sv_info->view_vector_to(position, jacobian(obs_index, 0), jacobian(obs_index, 1), jacobian(obs_index, 2), distance);
  observation(obs_index) =
      sig->pseudorange - distance - parameter(clock_index) + Constants::CLIGHT * sv_info->dtsv - iono_err - trop_err;
  jacobian(obs_index, clock_index) = 1;
  weight(obs_index, obs_index) =
      sig->code_var * sig->code_var;  // set weight (notice : here is the observation variance, not weight,
                                      // need to inverse the weight later
}

// this function is used to handle the velocity model
// - jacobian    (modeled)
// - observation (modeled)
// - weight      (unmodeled)
template <std::floating_point _Float_t>
void handle_spp_velocity_model(algorithm::WeightedLeastSquare<_Float_t>& wls,
                               const utils::CoordinateXyz& station_position, const EphemerisResult* sv_info,
                               f64 doppler, i32 doppler_index) {
  f64 distance = 0;
  auto& jacobian = wls.jacobian();
  auto& observation = wls.observation();
  jacobian.row(doppler_index).setZero();  // reset jacobian
  sv_info->view_vector_to(station_position, jacobian(doppler_index, 0), jacobian(doppler_index, 1),
                          jacobian(doppler_index, 2), distance);
  jacobian(doppler_index, 3) = 1;
  f64 rate_row0 = (jacobian(doppler_index, 0) * sv_info->vel.x() + jacobian(doppler_index, 1) * sv_info->vel.y() +
                   jacobian(doppler_index, 2) * sv_info->vel.z()) /
                  distance;
  observation(doppler_index) = doppler - rate_row0 + Constants::CLIGHT * sv_info->fd_dtsv;
}

bool Spp::load_spp_payload() noexcept {
  if (!rover_->update_record()) return false;
  (*this)
      ._set_information(rover_)                                                     // first set information
      ._set_obs_handler(rover_, task_config())                                      // set observation handler
      ._set_solution(sol_.get())                                                    // set solution to output
      ._set_atmosphere_error(_satellite_number())                                   // set atmosphere error
      ._set_wls(3 + _clock_parameter_number(), _signal_number(), rover_->logger())  // set wls
      ._handle_variance(rover_->settings()->random);                                // handle variance
  return true;
}

SppPayload& SppPayload::_set_information(std::shared_ptr<GnssHandler>& handler) noexcept {
  info_ = handler->update_runtime_info();
  return *this;
}

SppPayload& SppPayload::_set_obs_handler(std::shared_ptr<GnssHandler>& handler,
                                         const TaskConfig& task_config) noexcept {
  obs_handler_ =
      std::make_unique<ObsHandlerType>(handler->generate_rawobs_handler(task_config.filter().mask_filter.get()));
  return *this;
}

SppPayload& SppPayload::_set_clock_map(const std::shared_ptr<GnssHandler>& handler) noexcept {
  u8 index = 0;
  std::ranges::for_each(handler->record()->obs->code_map() | std::views::keys,
                        [&](ConstellationEnum cons) { clock_map_[cons] = index++; });
  return *this;
}

EpochUtc SppPayload::_epoch() const noexcept { return info_->epoch; }

bool SppPayload::_position_solvable() const noexcept {
  return info_ && obs_handler_ && _signal_number() >= 3 + _clock_parameter_number();
}

bool SppPayload::_velocity_solvable() const noexcept { return info_ && obs_handler_ && _satellite_number() >= 3 + 1; }

u16 SppPayload::_satellite_number() const noexcept { return static_cast<u16>(obs_handler_->size()); }

u16 SppPayload::_signal_number() const noexcept {
  u16 sig_num = 0;
  std::ranges::for_each(*obs_handler_, [&sig_num](const GnssRawObsHandler& handler) { sig_num += handler.sig.size(); });
  return sig_num;
}

void SppPayload::_handle_variance(sensors::gnss::RandomModelEnum model) const noexcept {
  std::ranges::for_each(*obs_handler_,
                        [model](const GnssRawObsHandler& handler) { handler.handle_signal_variance(model); });
}

SppPayload& SppPayload::_set_atmosphere_error(u16 number) noexcept {
  trop_error_ = std::make_unique<AtmosphereError>(number, 0);
  iono_error_ = std::make_unique<AtmosphereError>(number, 0);
  return *this;
}

SppPayload& SppPayload::_set_wls(int parameter_size, int observation_size,
                                 std::shared_ptr<spdlog::logger> logger) noexcept {
  wls_ = std::make_unique<algorithm::WeightedLeastSquare<f64>>(parameter_size, observation_size, logger);
  wls_->parameter().block(0, 0, 3, 1) = sol_->position;  // preset position
  return *this;
}

SppPayload& SppPayload::_set_solution(PvtSolutionRecord* sol) noexcept {
  sol_ = sol;
  sol_->time = info_->epoch;  // set solution time
  return *this;
}

auto SppPayload::_raw_obs_at(u16 index) const noexcept -> const sensors::gnss::GnssRawObsHandler& {
  return obs_handler_->at(index);
}

auto SppPayload::_trop_error_at(u16 index) const noexcept -> f64 { return trop_error_->at(index); }

auto SppPayload::_iono_error_at(u16 index) const noexcept -> f64 { return iono_error_->at(index); }

u8 SppPayload::_clock_parameter_number() const noexcept { return clock_map_.size(); }

u8 SppPayload::_clock_parameter_index(sensors::gnss::Sv sv) const noexcept { return clock_map_.at(sv.system()); }

void SppPayload::_calculate_atmosphere_error(TropModelEnum trop, IonoModelEnum iono) noexcept {
  for (u16 sat_index = 0; sat_index < obs_handler_->size(); ++sat_index) {
    auto& obs = obs_handler_->at(sat_index);
    obs.sv_info->update_ea_from(sol_->position);                  // update satellite elevation and azimuth
    (*trop_error_)[sat_index] = obs.trop_corr(&sol_->blh, trop);  // calculate trop error
    (*iono_error_)[sat_index] = obs.iono_corr(&sol_->blh, iono);  // calculate iono error
  }
}

f64 SppPayload::_position_iter_once() noexcept {
  wls_->correct();
  sol_->position = wls_->parameter().block(0, 0, 3, 1);
  sol_->blh = sol_->position.to_blh();
  return wls_->parameter_correction().block(0, 0, 3, 1).norm();
}

f64 SppPayload::_velocity_iter_once() noexcept {
  wls_->correct();
  return wls_->parameter_correction().block(0, 0, 3, 1).norm();
}

void SppPayload::_reset() noexcept {
  obs_handler_.reset();
  iono_error_.reset();
  trop_error_.reset();
  wls_.reset();
  info_ = nullptr;
  sol_ = nullptr;
}

void SppPayload::_position_evaluate() noexcept {
  wls_->evaluate();
  // position
  sol_->position = wls_->parameter().block(0, 0, 3, 1);
  sol_->blh = sol_->position.to_blh();
  auto& cofactor = wls_->cofactor();
  sol_->qr[0] = static_cast<f32>(cofactor(0, 0)), sol_->qr[1] = static_cast<f32>(cofactor(0, 1)),
  sol_->qr[2] = static_cast<f32>(cofactor(0, 2)), sol_->qr[3] = static_cast<f32>(cofactor(1, 1)),
  sol_->qr[4] = static_cast<f32>(cofactor(1, 2)), sol_->qr[5] = static_cast<f32>(cofactor(2, 2));
  sol_->sigma_r = wls_->sigma();
  // clock
  if (clock_map_.contains(ConstellationEnum::BDS)) {
    sol_->dtr[0] = wls_->parameter()(3 + clock_map_.at(ConstellationEnum::BDS));
  }
  if (clock_map_.contains(ConstellationEnum::GPS)) {
    sol_->dtr[1] = wls_->parameter()(3 + clock_map_.at(ConstellationEnum::GPS));
  }
  sol_->mode = SolutionModeEnum::SINGLE;  // mode
  sol_->type = 0;                         // type
  sol_->ns = obs_handler_->size();        // number of satellites

  wls_.reset();  // reset wls
}

void SppPayload::_velocity_evaluate() noexcept {
  wls_->evaluate();
  // velocity
  sol_->velocity = wls_->parameter().block(0, 0, 3, 1);
  auto& cofactor = wls_->cofactor();
  sol_->qv[0] = static_cast<f32>(cofactor(0, 0)), sol_->qv[1] = static_cast<f32>(cofactor(0, 1)),
  sol_->qv[2] = static_cast<f32>(cofactor(0, 2)), sol_->qv[3] = static_cast<f32>(cofactor(1, 1)),
  sol_->qv[4] = static_cast<f32>(cofactor(1, 2)), sol_->qv[5] = static_cast<f32>(cofactor(2, 2));
  sol_->sigma_v = wls_->sigma();
  wls_.reset();  // reset wls
}

Spp::Spp(std::string_view cfg_path, bool enabled_mt)
    : Task(cfg_path), rover_(config_.rover_station(enabled_mt)), sol_(std::make_unique<PvtSolutionRecord>()) {
  this->_set_clock_map(rover_);
}

auto Spp::solution() const noexcept -> const PvtSolutionRecord* { return sol_.get(); }

void Spp::model_spp_position() noexcept {
  u16 sig_index = 0;  // signal index
  auto& wls = _wls();
  for (u16 sat_index = 0; sat_index < _satellite_number(); ++sat_index) {
    auto& obs = _raw_obs_at(sat_index);
    auto sv_info = obs.sv_info;
    auto clock_index = _clock_parameter_index(sv_info->sv);
    for (auto sig : obs.sig) {
      handle_spp_signal_model(wls, sig, sv_info, _trop_error_at(sat_index), _iono_error_at(sat_index), sig_index++,
                              3 + clock_index);
    }
  }
  wls.weight() = wls.weight().inverse();  // inverse convariance to weight
}

void Spp::model_spp_velocity() noexcept {
  if (!_position_solvable()) return;
  auto sat_nums = _satellite_number();
  std::vector<f64> doppler(sat_nums, 0);
  for (u16 sat_index = 0; sat_index < sat_nums; ++sat_index) {
    u8 dopper_obs_num = 0;
    // average doppler on single frequency
    for (auto sig : _raw_obs_at(sat_index).sig) {
      doppler[sat_index] += sig->doppler * Constants::wave_length(sig->freq);
      ++dopper_obs_num;
    }
    doppler[sat_index] /= dopper_obs_num;
  }
  _set_wls(4, sat_nums, rover_->logger());
  for (u16 sat_index = 0; sat_index < sat_nums; ++sat_index) {
    auto sv_info = _raw_obs_at(sat_index).sv_info;
    handle_spp_velocity_model(_wls(), sol_->position, sv_info, doppler[sat_index], sat_index);
  }
  _wls().weight().setIdentity();  // set weight to identity matrix
}

bool Spp::solve_position() noexcept {
  if (!_position_solvable()) return false;
  u8 iteration = 0;  // iteration number
  bool atmosphere_corrected = false;
  while (true) {
    model_spp_position();
    auto position_correction = _position_iter_once();
    // when the position correction < 1, correct the atmosphere error
    if (!atmosphere_corrected && position_correction < 1) {
      _calculate_atmosphere_error(rover_->settings()->trop, rover_->settings()->iono);
      atmosphere_corrected = true;
    }
    // iteration end
    if (position_correction < 1e-6 || iteration > 9) {
      _position_evaluate();
      break;
    }
    ++iteration;
  }
  return true;
}

bool Spp::solve_velocity() noexcept {
  if (!_velocity_solvable()) return false;
  model_spp_velocity();
  auto velocity_correction = _velocity_iter_once();
  return true;
}

bool Spp::next_solution() noexcept {
  if (!load_spp_payload()) return false;
  bool done = false;
  done = solve_position();
  done = solve_velocity();
  return done;
}

}  // namespace navp::solution