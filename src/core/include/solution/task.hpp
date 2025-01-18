#pragma once

#include <solution/config.hpp>

#include "filter/filter.hpp"
#include "utils/macro.hpp"

namespace navp::solution {

struct NAVP_EXPORT TaskConfig {
  TaskConfig(NavConfigManger& config);

  struct Meta {
    std::string task_name;
    std::string project_name;
    EpochUtc execute_time;
    std::string executor;
  };

  struct Solution {
    SolutionModeEnum mode;
    algorithm::AlgorithmEnum algorithm;
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

 private:
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
  virtual void before_action() = 0;

  virtual void after_action() = 0;

  virtual void action() = 0;

  NavConfigManger config_;

 private:
  std::unique_ptr<TaskConfig> task_config_;
};

}  // namespace navp::solution