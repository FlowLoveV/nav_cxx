#include "sensors/gnss/gnss_handler.hpp"

#include <ranges>

#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/constants.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/random.hpp"

namespace navp::sensors::gnss {

using navp::sensors::gnss::GnssRandomHandler;

bool GnssRecord::update(GnssRuntimeInfo* runtime_info) {
  if (!obs_stream->eof()) [[likely]] {
    obs->get_record(*obs_stream);  // read next epoch observation
    return true;
  }
  return false;
}

void GnssRuntimeInfo::update(const GnssRecord* record) {
  auto [_epoch, _obs] = record->obs->latest();
  epoch = _epoch, obs_map = &_obs;
  avilable_sv = record->eph_solver->solve_sv_status(epoch, &_obs);
  sv_map = record->eph_solver->quary_sv_status(epoch);
}

// std::shared_ptr<solution::PvtSolutionRecord> GnssStationHandler::latest_spp_result() noexcept {
//   if (latest_obs_ == nullptr || latest_sv_map_ == nullptr) return nullptr;
//   AtmosphereHandlerMap trop_handler_map, iono_handler_map;
//   std::ranges::for_each(latest_avilable_sv_, [&](Sv sv) {
//     trop_handler_map.insert({sv, generate_trop_handler(sv)});
//     iono_handler_map.insert({sv, generate_iono_handler(sv)});
//   });

//   u16 observation_num;
//   std::unordered_map<Sv, std::list<const Sig*>> sv_sig_map;
//   std::ranges::for_each(*latest_obs_ | std::views::values, [&](const GnssObsRecord::ObsPtr& gobs) {
//     const auto& enabled_codes = obs_code_[gobs->sv.system()];
//     std::ranges::for_each(gobs->sigs_list | std::views::values | std::views::join, [&](const Sig& sig) {
//       if (enabled_codes.contains(sig.code) && sig.pseudorange != 0) {
//         sv_sig_map[gobs->sv].emplace_back(&sig);
//         observation_num++;
//       }
//     });
//   });

//   utils::NavVectorDf64 parameter(3 + clock_map_->size());
//   utils::NavVectorDf64 correction(3 + clock_map_->size());
//   utils::NavVectorDf64 residual(observation_num);
//   utils::NavMatrixDf64 jacobian(observation_num, 3 + clock_map_->size());
//   utils::NavMatrixDf64 weight(observation_num, observation_num);

// SppLoop: {
//   std::ranges::for_each(sv_sig_map, [](const auto& pair) {
//     Sv sv = pair.first;
//     std::ranges::for_each(pair.second, [&](const Sig* sig) {

//     });
//   });
//   for (u16 i = 0; i < observation_num; i++) {
//     auto sig = sig_vec[i];
//     auto sv = sv_vec[i];
//     auto& sv_info = latest_sv_map_->at(sv);
//     f64 distance = 0;
//     auto clock_index = clock_map_->at(sv.system());
//     sv.view_vector_to(latest_spp_result_->position, jacobian(i, 0), jacobian(i, 1), jacobian(i, 2), distance);
//     residual(i) = sig->pseudorange - distance - parameter(clock_index) + Constants::CLIGHT * sv_info.dtsv;

//     if (correction(0) != 0 && correction(0) < 1) {
//       sv_info.calculate_ea_from(latest_spp_result_->position);
//       latest_spp_result_->blh = latest_spp_result_->position.to_blh();
//       residual(i) -= trop_handler_map.at(sv)(&latest_spp_result_->blh);
//       residual(i) -= iono_handler_map.at(sv)(&latest_spp_result_->blh);
//     }
//   }
// }
//   u8 loop_iters = 0;
//   while (true) {
//     goto SppLoop;
//     loop_iters++;
//     if (loop_iters > 9 || correction.block(0, 0, 3, 1).norm() < 1e-6) break;
//   }

//   return latest_spp_result_;
// }

NAV_NODISCARD_UNUNSED auto GnssStationHandler::generate_rawobs_handler() const noexcept -> std::vector<RawObsHandler> {
  std::vector<RawObsHandler> handler;
  auto& satellites_vector = runtime_info_->avilable_sv;
  handler.reserve(satellites_vector.size());
  u16 sv_count = 0;
  for (u16 i = 0; i < satellites_vector.size(); i++) {
    if (!settings_->enabled(satellites_vector[i])) continue;  // filter sv and system
    GObs* obs = runtime_info_->obs_map->at(satellites_vector[i]).get();
    std::vector<const Sig*> sig;
    obs->for_each_code([sig = &sig, codes = settings_->enabled_codes(satellites_vector[i])](const Sig& _sig) mutable {
      if (codes.contains(_sig.code)) sig->push_back(&_sig);  // filter code
    });
    if (sig.empty()) continue;  // if no sigs in this obs, skip
    handler.emplace_back(obs, std::move(sig));
  }
  handler.shrink_to_fit();
  return handler;
}

// todo
NAV_NODISCARD_ERROR_HANDLE auto GnssStationHandler::generate_undiffobs_handler() const noexcept
    -> std::vector<UnDiffObsHandler> {
  auto& satellites_vector = runtime_info_->avilable_sv;
  std::vector<UnDiffObsHandler> handler(satellites_vector.size());
  for (auto sv : satellites_vector) {
    CombObsMeta meta;
    auto& obs = runtime_info_->obs_map->at(sv);
    switch (obs->frequency_count()) {
      case 1: {
      }
      case 2: {
      }
      default: {
      }
    }
  }
  return std::move(handler);
}

auto GnssStationHandler::generate_trop_handler(Sv sv) const noexcept
    -> std::function<f64(const utils::CoordinateBlh*)> {
  return [handler = static_cast<TropHandler>(
              TropHandler{}.set_time(runtime_info_->epoch).set_sv_info(&runtime_info_->sv_map->at(sv))),
          model = settings_->trop](const utils::CoordinateBlh* pos) mutable {
    handler.set_station_pos(pos);
    return handler.handle(model);
  };
}

auto GnssStationHandler::generate_iono_handler(Sv sv) const noexcept
    -> std::function<f64(const utils::CoordinateBlh*)> {
  return [handler = static_cast<IonoHandler>(
              IonoHandler{}.set_time(runtime_info_->epoch).set_sv_info(&runtime_info_->sv_map->at(sv))),
          model = settings_->iono](const utils::CoordinateBlh* pos) mutable {
    handler.set_station_pos(pos);
    return handler.handle(model);
  };
}

auto GnssStationHandler::generate_random_handler(Sv sv) const noexcept -> std::function<const Sig*(ObsCodeEnum)> {
  return [handler = GnssRandomHandler{}
                        .set_obs(runtime_info_->obs_map->at(sv).get())
                        .set_sv_info(&runtime_info_->sv_map->at(sv)),
          model = settings_->random](ObsCodeEnum code) { return handler.handle(code, model); };
}

}  // namespace navp::sensors::gnss