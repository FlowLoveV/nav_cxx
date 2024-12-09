#include "solution/task.hpp"

namespace navp::solution {

Task::Task() = default;

ConfigTask::~ConfigTask() = default;

ConfigTask::ConfigTask(std::string_view cfg_path) : config_(cfg_path) {}

void ConfigTask::export_config(std::ostream& os) const noexcept { os << config_; }

}  // namespace navp::solution