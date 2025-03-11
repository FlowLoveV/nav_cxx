#include "solution/rtk.hpp"

#include <algorithm>
#include <ranges>

#include "algorithm/ambiguity_fixer.hpp"
#include "sensors/gnss/constants.hpp"

namespace navp::solution {

using sensors::gnss::Constants;
using sensors::gnss::GnssRandomHandler;
using sensors::gnss::Sig;

Rtk::Rtk(const TaskConfig& task_config, bool enabled_mt)
    : ratio_threshold_(task_config.config().ratio()),
      logger_(task_config.logger()),
      rover_(std::make_unique<Spp>(task_config, enabled_mt)),
      base_(std::make_unique<Spp>(task_config, enabled_mt)),
      solution_(task_config.solution().capacity) {
  indicator_.base_fixed = base_->station()->station_info()->fixed;
  if (!indicator_.base_fixed) {
    logger_->warn("Base station \'{}\' is not fixed, using Base station spp result as reference",
                  base_->station()->station_info()->name);
  }
  __RtkPayload::_set_maskfilters(task_config);
}

bool Rtk::load_next_epoch() noexcept {
  solution_.push();
  return align_time();
}

bool Rtk::load_rtk_payload() noexcept {
  if (rover_->solve() && (indicator_.base_fixed || base_->solve())) {
    solution_.last() = *rover_->solution();  // set solution
    return __RtkPayload::_reset(rover_.get(), base_.get());
  }
  return false;
}

bool Rtk::align_time() noexcept {
  // pass the first epoch
  if (rover_->epoch() == base_->epoch() && rover_->epoch() == EpochUtc()) {
    rover_->load_next_epoch();
    base_->load_next_epoch();
    return align_time();
  }
  while (rover_->epoch() != base_->epoch()) {
    if (rover_->epoch() < base_->epoch()) {
      rover_->load_next_epoch();
      continue;
    } else {
      base_->load_next_epoch();
      continue;
    }
  }
  return rover_->epoch() == base_->epoch();
}

bool __RtkPayload::_reset(const Spp* rover, const Spp* base) noexcept {
  // claer old data
  system_payload_map_.clear();
  wls_.reset();
  // check if rover and base is valid
  if (!rover || !base) return false;
  // reset
  rover_ = rover->station(), base_ = base->station();
  rover_pos_ = std::addressof(rover->solution()->position);
  if (!indicator_.base_fixed) {
    base_pos_ = std::addressof(base->solution()->position);
  } else {
    base_pos_ = base->station()->station_info()->ref_pos.get();
  }
  // The order of the following operations cannot be changed
  if (!_get_public_view_satellites()) return false;
  _select_reference_satellite();
  _get_rover_information();
  _get_base_information();
  _select_available_sigs();
  if (_solvable()) {
    _handle_variance();
    return true;
  }
  return false;
}

auto __RtkPayload::epoch() const noexcept -> EpochUtc { return rover_->runtime_info()->epoch; }

bool __RtkPayload::_get_public_view_satellites() noexcept {
  if (mask_filter_ && !mask_filter_->apply(epoch())) return false;  // filter time
  auto base_obs_map = base_->runtime_info()->obs_map;
  auto rover_obs_map = rover_->runtime_info()->obs_map;
  for (const auto& [sv, _] : *base_obs_map) {
    if (mask_filter_ && (!mask_filter_->apply(sv.system()) || !mask_filter_->apply(sv)))
      continue;  // filter sv and system
    if (rover_obs_map->contains(sv)) {
      if (mask_filter_ && !mask_filter_->apply(filter::ElevationItem(rover_->runtime_info()->sv_map->at(sv).elevation)))
        continue;  // filter low elevation satellite
      system_payload_map_[sv.system()].public_view_satellites.emplace_back(sv);
    }
  }
  return true;
}

void __RtkPayload::_select_reference_satellite() noexcept {
  auto rover_sv_map = rover_->runtime_info()->sv_map;
  for (auto& [sys, payload] : system_payload_map_) {
    f64 max_elevation = rover_sv_map->at(payload.public_view_satellites[0]).elevation;
    for (u8 i = 1; i < payload.public_view_satellites.size(); ++i) {
      auto elevation = rover_sv_map->at(payload.public_view_satellites[i]).elevation;
      if (elevation >= max_elevation) {
        max_elevation = elevation;
        // always keep the max elevation satellite at first
        std::swap(payload.public_view_satellites[0], payload.public_view_satellites[i]);
      }
    }
    // sort the rest satellite list
    std::sort(payload.public_view_satellites.begin() + 1, payload.public_view_satellites.end());
  }
}

void __RtkPayload::_get_rover_information() noexcept {
  std::ranges::for_each(system_payload_map_ | std::views::values, [rover = rover_](SystemPayload& payload) {
    payload.rover_eph.reserve(payload.public_view_satellites.size());
    for (auto sv : payload.public_view_satellites) {
      payload.rover_eph.emplace_back(&rover->runtime_info()->sv_map->at(sv));
    }
  });
}

void __RtkPayload::_get_base_information() noexcept {
  std::ranges::for_each(system_payload_map_ | std::views::values, [base_pos = base_pos_](SystemPayload& payload) {
    payload.bt_base_satellites_distance.reserve(payload.public_view_satellites.size());
    for (auto eph : payload.rover_eph) {
      payload.bt_base_satellites_distance.emplace_back((*base_pos - eph->pos).norm());
    }
  });
}

void __RtkPayload::_handle_variance() noexcept {
  std::ranges::for_each(system_payload_map_ | std::views::values, [&](SystemPayload& payload) {
    payload.handle_variance(rover_->settings()->random, base_->settings()->random);
  });
}

void __RtkPayload::_select_available_sigs() noexcept {
  std::ranges::for_each(system_payload_map_ | std::views::values,
                        [&](SystemPayload& payload) { payload.select_available_sigs(rover_, base_, mask_filter_); });
}

void __RtkPayload::SystemPayload::select_available_sigs(const GnssHandler* rover, const GnssHandler* base,
                                                        const filter::MaskFilters* mask_filter) noexcept {
  if (public_view_satellites.size() < 2) return;  // if public view satellites less than 2, return
  auto& rover_obs_map = rover->runtime_info()->obs_map;
  auto& base_obs_map = base->runtime_info()->obs_map;

  // get observation vector
  auto rover_obs_vec =
      std::views::transform(public_view_satellites, [&](const Sv sv) { return rover_obs_map->at(sv).get(); }) |
      std::ranges::to<std::vector>();
  auto base_obs_vec =
      std::views::transform(public_view_satellites, [&](const Sv sv) { return base_obs_map->at(sv).get(); }) |
      std::ranges::to<std::vector>();

  // initialize available code set
  rover_obs_vec[0]->for_each_code([&](const Sig& sig) {
    if (sig.is_valid(mask_filter)) {
      available_code_set.insert(sig.code);
    }
  });

  // delete code that is not available in the rest of the satellites
  for (u8 i = 1; i < public_view_satellites.size(); ++i) {
    auto obs_i = rover_obs_vec[i];
    for (auto code : available_code_set) {
      if (auto sig = obs_i->find_code(code); !sig || !sig->is_valid(mask_filter)) {
        available_code_set.erase(code);
      }
    }
  }
  for (u8 i = 0; i < public_view_satellites.size(); ++i) {
    auto obs_i = base_obs_vec[i];
    for (auto code : available_code_set) {
      if (auto sig = obs_i->find_code(code); !sig || !sig->is_valid(mask_filter)) {
        available_code_set.erase(code);
      }
    }
  }

  if (available_code_set.empty()) return;

  base_sigs.resize(public_view_satellites.size() * available_code_set.size());
  rover_sigs.resize(public_view_satellites.size() * available_code_set.size());

  for (u8 i = 0; i < public_view_satellites.size(); ++i) {
    auto& rover_obs_i = rover_obs_map->at(public_view_satellites[i]);
    auto& base_obs_i = base_obs_map->at(public_view_satellites[i]);
    for (auto [index, code] : std::views::zip(std::views::iota(0), available_code_set)) {
      rover_sigs[i + index * public_view_satellites.size()] = rover_obs_i->find_code(code);
      base_sigs[i + index * public_view_satellites.size()] = base_obs_i->find_code(code);
    }
  }
}

bool __RtkPayload::_solvable() const noexcept {
  u8 code_num = 0;
  std::ranges::for_each(system_payload_map_ | std::views::values,
                        [&code_num](const SystemPayload& payload) { code_num += payload.rover_sigs.size(); });
  return code_num >= 4;
}

__RtkPayload& __RtkPayload::_set_maskfilters(const TaskConfig& config) noexcept {
  mask_filter_ = config.filter().mask_filter.get();
  return *this;
}

__RtkPayload& __RtkPayload::_set_wls(size_t parameter_size, size_t observation_size,
                                     std::shared_ptr<spdlog::logger> logger) noexcept {
  wls_ = std::make_unique<algorithm::WeightedLeastSquare<f64>>(parameter_size, observation_size, logger);
  return *this;
}

u16 __RtkPayload::_bt_station_ambiguity_size() const noexcept {
  u16 size = 0;
  std::ranges::for_each(system_payload_map_ | std::views::values,
                        [&size](const SystemPayload& payload) { size += payload.bt_station_ambiguity_size(); });
  return size;
}

u16 __RtkPayload::_bt_satellite_ambiguity_size() const noexcept {
  u16 size = 0;
  std::ranges::for_each(system_payload_map_ | std::views::values,
                        [&size](const SystemPayload& payload) { size += payload.bt_satellite_ambiguity_size(); });
  return size;
}

u8 __RtkPayload::_dd_ambiguity_size() const noexcept {
  u8 size = 0;
  std::ranges::for_each(system_payload_map_ | std::views::values,
                        [&size](const SystemPayload& payload) { size += payload.dd_ambiguity_size(); });
  return size;
}

bool __RtkPayload::SystemPayload::is_view_vector_cached() const noexcept {
  return view_vector_cache && view_vector_cache->size() == public_view_satellites.size();
}

void __RtkPayload::SystemPayload::reset_view_vector_cache() const noexcept { view_vector_cache.reset(); }

void __RtkPayload::SystemPayload::update_view_vector_cache() const noexcept {
  reset_view_vector_cache();
  view_vector_cache = std::make_unique<ViewVectorCache>();
  view_vector_cache->reserve(public_view_satellites.size());
  for (u8 i = 0; i < public_view_satellites.size(); ++i) {
    view_vector_cache->emplace_back(rover_eph[i]->view_vector_to(rover_spp_sol_->position));
  }
}

bool __RtkPayload::SystemPayload::is_bt_sta_sd_obs_cached() const noexcept {
  return btsta_sd_obs_cache && btsta_sd_obs_cache->size() == bt_station_ambiguity_size();
}

void __RtkPayload::SystemPayload::update_bt_sta_sd_obs_cache() const noexcept {
  reset_bt_sta_sd_obs_cache();
  btsta_sd_obs_cache = std::make_unique<ObservationCacheVector>();
  btsta_sd_obs_cache->resize(bt_station_ambiguity_size());

  for (auto&& [index, code] : std::views::zip(std::views::iota(0), available_code_set)) {
    f64 lambda = Constants::code_to_wave_length(public_view_satellites[0].system(), code);
    for (u8 i = 0; i < public_view_satellites.size(); ++i) {
      auto sig_index = index * public_view_satellites.size() + i;
      auto rover_sig = rover_sigs[sig_index];
      auto base_sig = base_sigs[sig_index];
      (*btsta_sd_obs_cache)[sig_index] = ObservationCache{
          .pseudorange = rover_sig->pseudorange - base_sig->pseudorange,
          .carrier = (rover_sig->carrier - base_sig->carrier) * lambda,
      };
    }
  }
}

void __RtkPayload::SystemPayload::reset_bt_sta_sd_obs_cache() const noexcept { btsta_sd_obs_cache.reset(); }

bool __RtkPayload::SystemPayload::is_bt_sat_sd_obs_cached() const noexcept {
  return rover_btsat_sd_obs_cache && base_btsat_sd_obs_cache &&
         rover_btsat_sd_obs_cache->size() == bt_satellite_ambiguity_size() &&
         base_btsat_sd_obs_cache->size() == bt_satellite_ambiguity_size();
}

void __RtkPayload::SystemPayload::update_bt_sat_sd_obs_cache() const noexcept {
  reset_bt_sat_sd_obs_cache();
  rover_btsat_sd_obs_cache = std::make_unique<ObservationCacheVector>();
  base_btsat_sd_obs_cache = std::make_unique<ObservationCacheVector>();
  rover_btsat_sd_obs_cache->resize(bt_satellite_ambiguity_size());
  base_btsat_sd_obs_cache->resize(bt_satellite_ambiguity_size());
  for (auto&& [index, code] : std::views::zip(std::views::iota(0), available_code_set)) {
    f64 lambda = Constants::code_to_wave_length(public_view_satellites[0].system(), code);
    for (u8 i = 1; i < public_view_satellites.size(); ++i) {
      auto ref_sv_index = index * public_view_satellites.size();
      auto rover_ref_sig = rover_sigs[ref_sv_index];
      auto base_ref_sig = base_sigs[ref_sv_index];
      auto rover_mov_sig = rover_sigs[ref_sv_index + i];
      auto base_mov_sig = base_sigs[ref_sv_index + i];
      (*rover_btsat_sd_obs_cache)[ref_sv_index + i - 1] = ObservationCache{
          .pseudorange = rover_mov_sig->pseudorange - rover_ref_sig->pseudorange,
          .carrier = (rover_mov_sig->carrier - rover_ref_sig->carrier) * lambda,
      };
      (*base_btsat_sd_obs_cache)[ref_sv_index + i - 1] = ObservationCache{
          .pseudorange = base_mov_sig->pseudorange - base_ref_sig->pseudorange,
          .carrier = (base_mov_sig->carrier - base_ref_sig->carrier) * lambda,
      };
    }
  }
}

void __RtkPayload::SystemPayload::reset_bt_sat_sd_obs_cache() const noexcept {
  rover_btsat_sd_obs_cache.reset();
  base_btsat_sd_obs_cache.reset();
}

bool __RtkPayload::SystemPayload::is_dd_obs_cached() const noexcept {
  return dd_obs_cache && dd_obs_cache->size() == dd_ambiguity_size();
}

void __RtkPayload::SystemPayload::update_dd_obs_cache() const noexcept {
  reset_dd_obs_cache();
  // between station single difference observation cache exists
  if (!is_bt_sta_sd_obs_cached()) update_bt_sta_sd_obs_cache();
  dd_obs_cache = std::make_unique<ObservationCacheVector>();
  dd_obs_cache->resize(dd_ambiguity_size());
  for (auto code_index : std::views::iota(0, (i32)available_code_set.size())) {
    auto ref_sv_index = code_index * public_view_satellites.size();
    auto& ref_sd_obs = btsta_sd_obs_cache->at(ref_sv_index);
    for (auto i = 1; i < btsta_sd_obs_cache->size(); ++i) {
      auto& rover_sd_obs = btsta_sd_obs_cache->at(ref_sv_index + i);
      (*dd_obs_cache)[ref_sv_index + i - 1] = (ObservationCache{
          .pseudorange = rover_sd_obs.pseudorange - ref_sd_obs.pseudorange,
          .carrier = rover_sd_obs.carrier - ref_sd_obs.carrier,
      });
    }
  }
}

void __RtkPayload::SystemPayload::reset_dd_obs_cache() const noexcept { dd_obs_cache.reset(); }

bool __RtkPayload::SystemPayload::is_bt_sta_sd_random_cached() const noexcept {
  return btsta_sd_random_cache && btsta_sd_random_cache->size() == bt_station_ambiguity_size();
}

void __RtkPayload::SystemPayload::update_bt_sta_sd_random_cache() const noexcept {
  reset_bt_sta_sd_random_cache();
  btsta_sd_random_cache = std::make_unique<ObservationCacheVector>();
  btsta_sd_random_cache->resize(bt_station_ambiguity_size());
  for (auto code_index : std::views::iota(0, (i32)available_code_set.size())) {
    auto ref_sv_index = code_index * public_view_satellites.size();
    for (u8 i = 0; i < public_view_satellites.size(); ++i) {
      auto sig_index = ref_sv_index + i;
      auto rover_sig = rover_sigs[sig_index];
      auto base_sig = base_sigs[sig_index];
      (*btsta_sd_random_cache)[sig_index] = ObservationCache{
          .pseudorange = rover_sig->code_var + base_sig->code_var,
          .carrier = rover_sig->phase_var + base_sig->phase_var,
      };
    }
  }
}

void __RtkPayload::SystemPayload::reset_bt_sta_sd_random_cache() const noexcept { btsta_sd_random_cache.reset(); }

bool __RtkPayload::SystemPayload::is_bt_sat_sd_random_cached() const noexcept {
  return rover_btsat_sd_random_cache && base_btsat_sd_random_cache &&
         rover_btsat_sd_random_cache->size() == bt_satellite_ambiguity_size() &&
         base_btsat_sd_random_cache->size() == bt_satellite_ambiguity_size();
}

void __RtkPayload::SystemPayload::update_bt_sat_sd_random_cache() const noexcept {
  reset_bt_sat_sd_random_cache();
  rover_btsat_sd_random_cache = std::make_unique<ObservationCacheVector>();
  base_btsat_sd_random_cache = std::make_unique<ObservationCacheVector>();
  rover_btsat_sd_random_cache->resize(bt_satellite_ambiguity_size());
  base_btsat_sd_random_cache->resize(bt_satellite_ambiguity_size());
  for (auto code_index : std::views::iota(0, (i32)available_code_set.size())) {
    auto ref_sv_index = code_index * public_view_satellites.size();
    for (u8 i = 1; i < public_view_satellites.size(); ++i) {
      auto mov_sv_index = ref_sv_index + i;
      auto rover_ref_sig = rover_sigs[ref_sv_index];
      auto base_ref_sig = base_sigs[ref_sv_index];
      auto rover_mov_sig = rover_sigs[mov_sv_index];
      auto base_mov_sig = base_sigs[mov_sv_index];
      (*rover_btsat_sd_random_cache)[mov_sv_index - 1] = ObservationCache{
          .pseudorange = rover_mov_sig->code_var + rover_ref_sig->code_var,
          .carrier = rover_mov_sig->phase_var + rover_ref_sig->phase_var,
      };
      (*base_btsat_sd_random_cache)[mov_sv_index - 1] = ObservationCache{
          .pseudorange = base_mov_sig->code_var + base_ref_sig->code_var,
          .carrier = base_mov_sig->phase_var + base_ref_sig->phase_var,
      };
    }
  }
}

void __RtkPayload::SystemPayload::reset_bt_sat_sd_random_cache() const noexcept {
  rover_btsat_sd_random_cache.reset();
  base_btsat_sd_random_cache.reset();
}

bool __RtkPayload::SystemPayload::is_dd_weight_cached() const noexcept {
  return dd_weight_cache && dd_weight_cache->rows() == dd_weight_cache->cols() &&
         dd_weight_cache->rows() == 2 * dd_ambiguity_size();
}

void __RtkPayload::SystemPayload::update_dd_weight_cache() const noexcept {
  reset_dd_weight_cache();
  // between station single difference random cache exists
  if (!is_bt_sta_sd_random_cached()) update_bt_sta_sd_random_cache();
  auto observation_size = 2 * dd_ambiguity_size();
  dd_weight_cache = std::make_unique<utils::NavMatrixDf64>(observation_size, observation_size);
  dd_weight_cache->setZero();
  std::size_t index = 0;
  std::size_t dd_sats = public_view_satellites.size() - 1;
  for (auto code_index : std::views::iota(0, (i32)available_code_set.size())) {
    auto ref_sv_index = code_index * dd_sats;
    auto& ref_sv_random = btsta_sd_random_cache->at(ref_sv_index);
    Eigen::Block<utils::NavMatrixDf64> pseudorange_variance =
        dd_weight_cache->block(ref_sv_index, ref_sv_index, dd_sats, dd_sats);
    Eigen::Block<utils::NavMatrixDf64> carrier_variance =
        dd_weight_cache->block(ref_sv_index + dd_sats, ref_sv_index + dd_sats, dd_sats, dd_sats);
    pseudorange_variance.array() = ref_sv_random.pseudorange;
    carrier_variance.array() = ref_sv_random.carrier;
    for (u8 i = 0; i < dd_sats; ++i) {
      auto mov_sv_index = ref_sv_index + i;
      auto mov_sv_random = btsta_sd_random_cache->at(ref_sv_index + i);
      pseudorange_variance(i, i) += mov_sv_random.pseudorange;
      carrier_variance(i, i) += mov_sv_random.carrier;
    }
  }
  dd_weight_cache->noalias() = dd_weight_cache->inverse();
}

void __RtkPayload::SystemPayload::reset_dd_weight_cache() const noexcept { dd_weight_cache.reset(); }

void __RtkPayload::SystemPayload::handle_variance(sensors::gnss::RandomModelEnum rover_model,
                                                  sensors::gnss::RandomModelEnum base_model) const noexcept {
  auto code_size = available_code_set.size();
  for (u16 i = 0; i < rover_sigs.size(); ++i) {
    auto random_handler =
        GnssRandomHandler{}.set_options(GnssRandomHandler::Both).set_sv_info(rover_eph[i / code_size]);
    random_handler.set_model(rover_model).handle(rover_sigs[i]);
    random_handler.set_model(base_model).handle(base_sigs[i]);
  }
}

void __RtkPayload::SystemPayload::build_dd_jacobian(Eigen::Block<utils::NavMatrixDf64> jacobian) const noexcept {
  if (!is_view_vector_cached()) update_view_vector_cache();
  auto& ref_vector = view_vector_cache->at(0);
  auto dd_sats = public_view_satellites.size() - 1;
  std::vector<f64> code_lambda(available_code_set.size());
  for (auto [code_index, code] : std::views::enumerate(available_code_set)) {
    code_lambda[code_index] = Constants::code_to_wave_length(public_view_satellites[0].system(), code);
  }
  for (u8 i = 0; i < dd_sats; ++i) {
    auto& rover_vector = view_vector_cache->at(i + 1);
    jacobian(i, 0) = ref_vector.x - rover_vector.x;
    jacobian(i, 1) = ref_vector.y - rover_vector.y;
    jacobian(i, 2) = ref_vector.z - rover_vector.z;
    jacobian(i + dd_sats, 0) = jacobian(i, 0);
    jacobian(i + dd_sats, 1) = jacobian(i, 1);
    jacobian(i + dd_sats, 2) = jacobian(i, 2);
    jacobian(i + dd_sats, 3 + i) = code_lambda[0];
  }

  for (u8 i = 1; i < available_code_set.size(); ++i) {
    auto row_index = 2 * i * dd_sats;
    for (u8 j = 0; j < dd_sats; ++j) {
      auto& rover_vector = view_vector_cache->at(j);
      jacobian(row_index + j, 0) = jacobian(j, 0);
      jacobian(row_index + j, 1) = jacobian(j, 1);
      jacobian(row_index + j, 2) = jacobian(j, 2);
      jacobian(row_index + j + dd_sats, 0) = jacobian(j + dd_sats, 0);
      jacobian(row_index + j + dd_sats, 1) = jacobian(j + dd_sats, 1);
      jacobian(row_index + j + dd_sats, 2) = jacobian(j + dd_sats, 2);
      jacobian(row_index + j + dd_sats, 3 + j) = code_lambda[i];
    }
  }
}

void __RtkPayload::SystemPayload::build_dd_observation(Eigen::Block<utils::NavVectorDf64> observation) const noexcept {
  if (!is_view_vector_cached()) update_view_vector_cache();
  if (!is_dd_obs_cached()) update_dd_obs_cache();
  f64 rover_ref_rho = view_vector_cache->at(0).distance;
  f64 base_ref_rho = bt_base_satellites_distance[0];
  auto dd_sats = public_view_satellites.size() - 1;
  for (auto code_index = 0; code_index < available_code_set.size(); ++code_index) {
    auto code_begin_index = code_index * (public_view_satellites.size() - 1);
    for (u8 i = 0; i < dd_sats; ++i) {
      auto mov_index = i + 1;
      f64 rover_btsta_rho = view_vector_cache->at(mov_index).distance - rover_ref_rho;
      f64 base_btsta_rho = bt_base_satellites_distance[mov_index] - base_ref_rho;
      f64 dd_rho = base_btsta_rho - rover_btsta_rho;
      observation(code_begin_index + i, 0) = dd_obs_cache->at(i).pseudorange - dd_rho;
      observation(code_index + i + dd_sats, 0) = dd_obs_cache->at(i).carrier - dd_rho;
    }
  }
}

void __RtkPayload::SystemPayload::build_dd_weight(Eigen::Block<utils::NavMatrixDf64> weight) const noexcept {
  if (!is_dd_weight_cached()) update_dd_weight_cache();
  weight = dd_weight_cache->inverse();
}

void __RtkPayload::SystemPayload::build_dd_model(algorithm::WeightedLeastSquare<f64>* wls,
                                                 u16 dd_ambiguity_index) const noexcept {
  auto ambiguous_size = dd_ambiguity_size();
  // build H
  build_dd_jacobian(wls->jacobian().block(2 * dd_ambiguity_index, 0, 2 * ambiguous_size, 3 + ambiguous_size));
  // build L
  build_dd_observation(wls->observation().block(2 * dd_ambiguity_index, 0, 2 * ambiguous_size, 1));
  // build R
  build_dd_weight(
      wls->weight().block(2 * dd_ambiguity_index, 2 * dd_ambiguity_index, 2 * ambiguous_size, 2 * ambiguous_size));
}

void __RtkPayload::_build_dd_model() noexcept {
  u16 dd_ambiguity_index = 0;
  for (auto& [sys, payload] : system_payload_map_) {
    payload.build_dd_model(wls_.get(), dd_ambiguity_index);
    dd_ambiguity_index += payload.dd_ambiguity_size();
  }
}

f64 __RtkPayload::_iter_once() noexcept {
  wls_->correct();
  return wls_->parameter_correction().block(0, 0, 3, 1).norm();
}

void __RtkPayload::_update_rover_position(const utils::CoordinateXyz* pos) noexcept {
  rover_pos_ = pos;
  // reset rover view vector cache
  std::ranges::for_each(__RtkPayload::system_payload_map_ | std::views::values,
                        [](const __RtkPayload::SystemPayload& payload) { payload.reset_view_vector_cache(); });
}

void __RtkPayload::_fix_ambiguity(f32 ratio_threshold) noexcept {
  auto float_baseline = (*rover_pos_ - *base_pos_);
  auto am_fixer = algorithm::AmbiguityFixer(float_baseline, wls_->parameter().data() + 3, wls_->cofactor());
  if (am_fixer.fix()) {
    if (am_fixer.ratio() >= ratio_threshold) {
      // todo
    }
  }
}

void Rtk::update_position_after_iter() noexcept {
  auto& sol = solution_.last();
  sol.position += __RtkPayload::wls_->parameter().block(0, 0, 1, 3);
  _update_rover_position(std::addressof(sol.position));
}

void Rtk::model(RtkModel model) noexcept {
  switch (model) {
    case DdBasic:
      model_dd_basic();
    default:
      return;
  }
}

void Rtk::evaluate() noexcept {}

void Rtk::model_dd_basic() noexcept { __RtkPayload::_build_dd_model(); }

bool Rtk::solve(RtkModel model) noexcept {
  // set wls
  size_t dd_ambiguity_size = __RtkPayload::_dd_ambiguity_size();
  __RtkPayload::_set_wls(3 + dd_ambiguity_size, 2 * dd_ambiguity_size, logger_);
  while (true) {
    Rtk::model(model);
    auto position_correction = __RtkPayload::_iter_once();
    update_position_after_iter();
    if (position_correction < 1e-6) {
    }
  }
}

RtkServer::RtkServer(std::string_view cfg_path, bool enabled_mt)
    : Task(cfg_path), Rtk(TaskConfig(cfg_path), enabled_mt) {}

}  // namespace navp::solution