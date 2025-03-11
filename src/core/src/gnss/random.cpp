#include "sensors/gnss/random.hpp"

#include "sensors/gnss/constants.hpp"

namespace navp::sensors::gnss {

namespace details {

// stardand random model
constexpr f32 stardand_pseudorange_var = 1.0f * 1.0f;
constexpr f32 stardand_carrier_var = 0.02f * 0.02f;

// elevation based random model

// snr based random model

// custom model

}  // namespace details

#define HANDLE_PSEUDORANGE(code)                       \
  if (options_ & EvaluateRandomOptions::Pseudorange) { \
    code;                                              \
  }

#define HANDLE_CARRIER(code)                       \
  if (options_ & EvaluateRandomOptions::Carrier) { \
    code;                                          \
  }

GnssRandomHandler& GnssRandomHandler::set_model(RandomModelEnum model) noexcept {
  model_ = model;
  return *this;
}

GnssRandomHandler& GnssRandomHandler::set_options(EvaluateRandomOptions options) noexcept {
  options_ = options;
  return *this;
}

GnssRandomHandler& GnssRandomHandler::set_sv_info(const EphemerisResult* eph_result) noexcept {
  sv_info_ = eph_result;
  return *this;
}

const Sig* GnssRandomHandler::handle(const Sig* sig) const noexcept {
  if (!sig) return nullptr;
  auto sig_ = const_cast<Sig*>(sig);
  if (sig_) {
    switch (model_) {
      case RandomModelEnum::STANDARD: {
        HANDLE_PSEUDORANGE(sig_->code_var = details::stardand_pseudorange_var);
        HANDLE_CARRIER(sig_->phase_var = details::stardand_carrier_var);
        return sig_;
      }
      case RandomModelEnum::ELEVATION_DEPENDENT: {
        return sig_;
      }
      case RandomModelEnum::SNR_DEPENDENT: {
        return sig_;
      }
      case RandomModelEnum::CUSTOM: {
        return sig_;
      }
      default: {
        nav_error("Encountering an unexpected branch in RandomModelEnum");
        return nullptr;
      }
    }
  }
  return nullptr;
}

#undef HANDLE_PSEUDORANGE
#undef HANDLE_CARRIER

}  // namespace navp::sensors::gnss