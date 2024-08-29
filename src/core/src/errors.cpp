#include "errors.hpp"

#include <cstdint>
#include <string_view>

#include "logger.hpp"
#include "spdlog/common.h"

namespace nav {

std::string_view ErrorMessage(ErrorId id) noexcept {
  switch (id.code) {
    case ErrorId::DividZero:
      return "Divid Zero";
    case ErrorId::InvSingularMatrix:
      return "Inverse a singular matrix";
    default:
      return "Default Error";
  }
}

void ErrorBase::warn(std::source_location location) {
  nav_log(spdlog::level::warn, "{} {}", error_msg, additional_msg);
}

void ErrorBase::crash(std::source_location location) {
  nav_log(spdlog::level::err, "{} {}", error_msg, additional_msg);
}

uint16_t NavError::warn(std::source_location location) {
  ErrorBase::warn(location);
  return static_cast<uint16_t>(error_id.code);
}

uint16_t NavError::crash(std::source_location location) {
  ErrorBase::crash(location);
  return static_cast<uint16_t>(error_id.code);
}

}  // namespace nav