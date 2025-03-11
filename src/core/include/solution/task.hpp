#pragma once

#include <solution/config.hpp>

#include "filter/filter.hpp"
#include "utils/macro.hpp"

namespace navp::solution {

struct NAVP_EXPORT TaskConfig {
  TaskConfig(std::string_view cfg_path);

  struct Meta {
    std::string task_name;
    std::string project_name;
    EpochUtc execute_time;
    std::string executor;
  };

  struct Solution {
    SolutionModeEnum mode;
    algorithm::AlgorithmEnum algorithm;
    i32 capacity;
  };

  struct Output {
    std::string output_dir;
  };

  struct Filter {
    std::unique_ptr<filter::MaskFilters> mask_filter;
  };

  inline auto meta() const noexcept -> const TaskConfig::Meta& { return __meta; }
  inline auto solution() const noexcept -> const TaskConfig::Solution& { return __solution; }
  inline auto filter() const noexcept -> const TaskConfig::Filter& { return __filter; }
  inline auto output() const noexcept -> const TaskConfig::Output& { return __output; }
  inline auto config() const noexcept -> const NavConfigManger& { return config_; }
  inline auto logger() const noexcept -> std::shared_ptr<spdlog::logger> { return config_.logger(); }
  inline auto rover_station(bool enabled_mt = false) const noexcept -> std::shared_ptr<sensors::gnss::GnssHandler> {
    return config_.rover_station(enabled_mt);
  }
  inline auto base_station(bool enabled_mt = false) const noexcept -> std::shared_ptr<sensors::gnss::GnssHandler> {
    return config_.rover_station(enabled_mt);
  }

 private:
  NavConfigManger config_;
  Meta __meta;
  Solution __solution;
  Output __output;
  Filter __filter;
};

class NAVP_EXPORT Task {
 public:
  Task(std::string_view cfg_path);

  virtual ~Task() = default;

  void run();

  inline const TaskConfig& task_config() { return *task_config_; }

  Task(const Task&) = delete;
  Task(Task&&) = default;

  Task& operator=(const Task&) = delete;
  Task& operator=(Task&&) = default;

 protected:
  virtual void before_action();

  virtual void after_action();

  virtual void action() = 0;

 private:
  std::unique_ptr<TaskConfig> task_config_;
};

}  // namespace navp::solution