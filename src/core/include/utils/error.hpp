#pragma once

#include "types.hpp"

namespace navp::errors {

enum class TopErrorEnum : u8 {
  InitializeError,
  ReadConfigError,
  ReadInputError,
  ProcessError,
  OutputError,
  UnknownError,
};

struct nav_err_code {
  u32 top : 4;
  u32 mid : 8;
  u32 details : 24;

  virtual std::string err_name(nav_err_code code) noexcept = 0;
};

struct nav_err : public nav_err_code, public std::exception {};

}  // namespace navp