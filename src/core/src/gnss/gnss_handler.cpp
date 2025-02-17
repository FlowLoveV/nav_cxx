#include "sensors/gnss/gnss_handler.hpp"

#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/random.hpp"

namespace navp::sensors::gnss {

using navp::sensors::gnss::GnssRandomHandler;

bool GnssRecord::update() {
  if (!obs_stream->eof()) [[likely]] {
    obs->get_record(*obs_stream);  // read next epoch observation
    return true;
  }
  return false;
}

void GnssRuntimeInfo::update(const GnssRecord* record) {
  auto& [_epoch, _obs] = record->obs->latest();
  epoch = _epoch;                                                   // epoch assgin
  obs_map = &_obs;                                                  // observation map
  avilable_sv = record->eph_solver->solve_sv_status(epoch, &_obs);  // available satellites
  sv_map = record->eph_solver->quary_sv_status(epoch);              // satellites map
}

// todo
NAV_NODISCARD_UNUNSED auto GnssPayload::generate_rawobs_handler(const filter::MaskFilters* mask_filter) const
    -> std::vector<GnssRawObsHandler> {
  std::vector<GnssRawObsHandler> handler;
  if (mask_filter && !mask_filter->apply(runtime_info_->epoch)) {
    return handler;  // if epoch mask filter not pass, return empty handler
  }
  auto& satellites_vector = runtime_info_->avilable_sv;
  handler.reserve(satellites_vector.size());
  u16 sv_count = 0;
  for (u16 i = 0; i < satellites_vector.size(); ++i) {
    if (mask_filter && !mask_filter->apply(satellites_vector[i].constellation)) {
      continue;  // if constellation mask filter not pass, skip
    }
    if (mask_filter && !mask_filter->apply(satellites_vector[i])) {
      continue;  // if sv mask filter not pass, skip
    }
    Sv sv = satellites_vector[i];
    if (!settings_->enabled(sv)) continue;  // filter unabled sv and system
    GObs* obs = runtime_info_->obs_map->at(sv).get();
    auto sv_info = &runtime_info_->sv_map->at(sv);
    if (mask_filter) {
      // if elevation mask filter not pass, skip
      if (sv_info->elevation != 0.0 && !mask_filter->apply(filter::ElevationItem(sv_info->elevation))) {
        continue;
      }
      // if azimuth mask filter not pass, skip
      if (sv_info->azimuth != 0.0 && !mask_filter->apply(filter::AzimuthItem(sv_info->azimuth))) {
        continue;
      }
    }
    std::vector<const Sig*> sig;
    obs->for_each_code([&](const Sig& _sig) {
      if (settings_->enabled(sv, _sig) && !_sig.invalid) {    // filter unabled code
        if (mask_filter->apply(filter::SnrItem(_sig.snr))) {  // filter snr
          sig.push_back(&_sig);
        }
      }
    });
    if (sig.empty()) continue;  // if no sigs in this obs, skip
    handler.emplace_back(obs, sv_info, std::move(sig));
  }
  handler.shrink_to_fit();
  return handler;
}

// todo
NAV_NODISCARD_ERROR_HANDLE auto GnssPayload::generate_undiffobs_handler() const -> std::vector<UnDiffObsHandler> {
  auto& satellites_vector = runtime_info_->avilable_sv;
  std::vector<UnDiffObsHandler> handler(satellites_vector.size());

  return std::move(handler);
}

auto GnssPayload::generate_atmosphere_handler(Sv sv) const -> AtmosphereHandler {
  return AtmosphereHandler{}
      .set_time(runtime_info_->epoch)
      .set_sv_info(&runtime_info_->sv_map->at(sv))
      .set_trop_model(settings_->trop)
      .set_iono_model(settings_->iono);
}

auto GnssPayload::generate_random_handler(Sv sv) const -> GnssRandomHandler {
  return GnssRandomHandler{}.set_model(settings_->random).set_sv_info(&runtime_info_->sv_map->at(sv));
}

void GnssRawObsHandler::handle_signal_variance(RandomModelEnum model,
                                               GnssRandomHandler::EvaluateRandomOptions options) const noexcept {
  auto random_handler = GnssRandomHandler{}.set_model(model).set_options(options).set_sv_info(sv_info);
  for (auto _sig : sig) {
    auto sig = random_handler.handle(_sig);
  }
}

f64 GnssRawObsHandler::trop_corr(const utils::CoordinateBlh* station_pos, TropModelEnum model) const noexcept {
  return AtmosphereHandler{}
      .set_time(static_cast<EpochUtc>(obs->time))
      .set_sv_info(sv_info)
      .set_trop_model(model)
      .handle_trop(station_pos);
}

f64 GnssRawObsHandler::iono_corr(const utils::CoordinateBlh* station_pos, IonoModelEnum model) const noexcept {
  return AtmosphereHandler{}
      .set_time(static_cast<EpochUtc>(obs->time))
      .set_sv_info(sv_info)
      .set_iono_model(model)
      .handle_iono(station_pos);
}

// todo
void GnssPayload::decode_header(io::Fstream& stream) const noexcept {
  auto obs_file = std::format("{} obs file  : {}", io::Fstream::AnnotationSymbols, record_->obs_stream->filename);
}

}  // namespace navp::sensors::gnss