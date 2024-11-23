#include "solution/task.hpp"

namespace navp::solution {

Task::Task() = default;

ConfigTask::~ConfigTask() = default;

ConfigTask::ConfigTask(std::string_view cfg_path) : config_(toml::parse_file(cfg_path)) {}

}  // namespace navp::solution