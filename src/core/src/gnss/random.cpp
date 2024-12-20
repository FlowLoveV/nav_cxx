#include "sensors/gnss/random.hpp"

#include "sensors/gnss/constants.hpp"

namespace navp::sensors::gnss {

namespace details {

// stardand random model
constexpr f32 stardand_pseudorange_var = 1.0f;
constexpr f32 stardand_carrier_var = 0.02f;

// elevation based random model

// snr based random model

// custom model

}  // namespace details

GnssRandomHandler& GnssRandomHandler::set_obs(const GObs* obs) noexcept {
  // safe const cast
  obs_ = const_cast<GObs*>(obs);
  return *this;
}

GnssRandomHandler& GnssRandomHandler::set_sv_info(const EphemerisResult* eph_result) noexcept {
  sv_info_ = eph_result;
  return *this;
}

const Sig* GnssRandomHandler::handle(ObsCodeEnum code, RandomModelEnum model) const noexcept {
  if (!obs_) return nullptr;
  auto sig = const_cast<Sig*>(obs_->find_code(code));  // safe and necessary const_cast here
  if (sig) {
    switch (model) {
      case RandomModelEnum::STANDARD: {
        sig->code_var = details::stardand_pseudorange_var;
        sig->phase_var = details::stardand_carrier_var;
        return sig;
      }
      case RandomModelEnum::ELEVATION_DEPENDENT: {
        return sig;
      }
      case RandomModelEnum::SNR_DEPENDENT: {
        return sig;
      }
      case RandomModelEnum::CUSTOM: {
        return sig;
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