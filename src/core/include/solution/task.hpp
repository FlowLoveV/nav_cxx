#pragma once

#include <iostream>
#include <solution/config.hpp>

#include "utils/macro.hpp"

namespace navp::solution {

class Task;

class NAVP_EXPORT Task {
 public:
  Task();
  virtual ~Task() = 0;
};

class NAVP_EXPORT ConfigTask {
 public:
  virtual ~ConfigTask();

  ConfigTask(std::string_view cfg_path);

  ConfigTask(const ConfigTask&) = delete;
  ConfigTask(ConfigTask&&) = default;

  ConfigTask& operator=(const ConfigTask&) = delete;
  ConfigTask& operator=(ConfigTask&&) = default;

  void export_config(std::ostream& os = std::cout) const noexcept;

 protected:
  NavConfigManger config_;
};

class NAVP_EXPORT ConcurrentTask : public ConfigTask {
 public:
  using ConfigTask::ConfigTask;

 protected:
  std::mutex mutex_;
};

}  // namespace navp::solution