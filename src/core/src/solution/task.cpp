#include "solution/task.hpp"

#include <filesystem>

namespace navp::solution {

TaskConfig::TaskConfig(std::string_view cfg_path) : config_(cfg_path) {
  // meta
  __meta.task_name = config_.task_name();
  __meta.project_name = config_.proj_name();
  __meta.execute_time = config_.executor_time();
  __meta.executor = config_.executor_name();

  // solution
  __solution.mode = config_.solution_mode();
  __solution.algorithm = config_.algorithm();
  __solution.capacity = config_.capacity();

  // output
  __output.output_dir = config_.output_dir();
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
      std::make_unique<filter::MaskFilters>(filter::MaskFilters::from_str(config_.filters()).unwrap_throw());
}

Task::Task(std::string_view cfg_path) : task_config_(std::make_unique<TaskConfig>(cfg_path)) {}

void Task::before_action() {
  auto logger_ = task_config_->logger();
  logger_->info("Task: {} starts", task_config_->meta().task_name);
}

void Task::after_action() {
  auto logger_ = task_config_->logger();
  logger_->info("Task: {} ends", task_config_->meta().task_name);
}

void Task::run() {
  before_action();
  action();
  after_action();
}

}  // namespace navp::solution