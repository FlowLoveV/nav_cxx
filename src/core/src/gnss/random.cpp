#include "sensors/gnss/random.hpp"

#include "sensors/gnss/constants.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"

namespace navp::sensors::gnss {

namespace details {

// stardand random model
constexpr f32 stardand_pseudorange_var = 1.0f;
constexpr f32 stardand_carrier_var = 0.02f;

// elevation based random model

// snr based random model

// custom model

}  // namespace details

GnssRandomHandler& GnssRandomHandler::set_time(EpochUtc time) noexcept {
  time_ = time;
  return *this;
}

GnssRandomHandler& GnssRandomHandler::set_random_model(RandomModelEnum type) noexcept {
  type_ = type;
  return *this;
}

GnssRandomHandler& GnssRandomHandler::target_obs(const GnssObsRecord& obs_record) noexcept {
  // safe const cast
  obs_map_ = const_cast<std::map<Sv, std::shared_ptr<GObs>>*>(obs_record.query(time_));
  return *this;
}

GnssRandomHandler& GnssRandomHandler::target_eph_solver(const EphemerisSolver& eph_solver) noexcept {
  eph_result_ = eph_solver.quary_sv_status(time_);
  return *this;
}

bool GnssRandomHandler::handlable() const noexcept { return obs_map_ && eph_result_; }

const Sig* GnssRandomHandler::handle(Sv sv, ObsCodeEnum code) {
  if (!handlable()) return nullptr;
  auto freq = Constants::code_to_freq_enum(sv.constellation.id, code);
  auto& obs = (*obs_map_)[sv]->sigsLists[freq];
  auto sig = std::find_if(obs.begin(), obs.end(), [&](Sig& sig) { return sig.code == code; });
  if (sig != obs.end()) {
    switch (type_) {
      case RandomModelEnum::STANDARD: {
        sig->code_var = details::stardand_pseudorange_var;
        sig->phase_var = details::stardand_carrier_var;
        return std::addressof(*sig);
      }
      case RandomModelEnum::ELEVATION_DEPENDENT: {
        return std::addressof(*sig);
      }
      case RandomModelEnum::SNR_DEPENDENT: {
        return std::addressof(*sig);
      }
      case RandomModelEnum::CUSTOM: {
        return std::addressof(*sig);
      }
      default: {
        nav_error("Encountering an unexpected branch in RandomModelEnum");
        return nullptr;
      }
    }
  }
  return nullptr;
}

}  // namespace navp::sensors::gnss