#pragma once

#include <iostream>
#include <solution/config.hpp>

#include "utils/macro.hpp"

namespace navp::filter {
class MaskFilters;
}

namespace navp::solution {

class TaskConfig;
class Task;

struct NAVP_EXPORT TaskConfig {
  struct Meta {
    std::string task_name;
    std::string project_name;
    std::string execute_time;
    std::string executor;
  };

  struct Solution {
    SolutionModeEnum mode;
    std::unique_ptr<sensors::gnss::GnssHandler> base;
    std::unique_ptr<sensors::gnss::GnssHandler> rover;
  };

  struct Output {
    std::string output_path;
  };

  struct Filter {
    std::unique_ptr<filter::MaskFilters> filter;
  };

  Meta __meta;
  Solution __solution;
  Output __output;
  Filter __filter;
};

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
  std::unique_ptr<TaskConfig> config_task_;
};

class NAVP_EXPORT ConcurrentTask : public ConfigTask {
 public:
  using ConfigTask::ConfigTask;

 protected:
  std::mutex mutex_;
};

}  // namespace navp::solution