#include "solution/task.hpp"

#include <filesystem>

namespace navp::solution {

TaskConfig::TaskConfig(NavConfigManger& config) {
  // meta
  __meta.task_name = config.task_name();
  __meta.project_name = config.proj_name();
  __meta.execute_time = config.executor_time();
  __meta.executor = config.executor_name();

  // solution
  __solution.mode = config.solution_mode();
  __solution.algorithm = config.algorithm();

  // output
  __output.output_dir = config.output_dir();
  if (!std::filesystem::exists(__output.output_dir)) {
    try {
      std::filesystem::create_directories(__output.output_dir);
    } catch (const std::filesystem::filesystem_error& e) {
      throw std::format("Create output directory failed: {}", e.what());
    }
  } else if (!std::filesystem::is_directory(__output.output_dir)) {
    throw std::format("Output path exists but is not a directory: {}", __output.output_dir);
  }

  // filter
  __filter.mask_filter =
      std::make_unique<filter::MaskFilters>(filter::MaskFilters::from_str(config.filters()).unwrap_throw());
}

Task::Task(std::string_view cfg_path) : config_(cfg_path), task_config_(std::make_unique<TaskConfig>(config_)) {}

void Task::run() {
  before_action();
  action();
  after_action();
}

}  // namespace navp::solution