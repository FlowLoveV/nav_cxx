#pragma once

#include "utils/macro.hpp"

namespace navp::solution {

class Config;

class NAVP_EXPORT Config {
 public:
  Config();
  virtual ~Config() = 0;

  virtual void from(const char* path) = 0;

  virtual const Config* get() = 0;
};

}  // namespace navp::solution