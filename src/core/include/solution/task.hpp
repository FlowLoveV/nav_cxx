#pragma once

#include <toml++/toml.hpp>

#include "utils/macro.hpp"

namespace navp::solution {

class Task;

using Config = toml::parse_result;

class NAVP_EXPORT Task {
 public:
  Task();
  virtual ~Task() = 0;

  virtual void solve() = 0;
};

class NAVP_EXPORT ConfigTask {
 public:
  virtual ~ConfigTask();

  ConfigTask(std::string_view cfg_path);

  virtual void solve() = 0;

 protected:
  Config config_;
};

class NAVP_EXPORT ConcurrentTask : public ConfigTask {
 public:
  using ConfigTask::ConfigTask;

 protected:
  std::mutex mutex_;
};

}  // namespace navp::solution