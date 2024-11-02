#include <filesystem>

#include "rfl/json.hpp"
#include "solution/spp_config.hpp"
#include "utils/error.hpp"
#include "utils/logger.hpp"

#define CHECK_FILE_EXIST(path)                                             \
  if (!std::filesystem::exists(path)) {                                    \
    nav_error("No configuration file found at {}", path);                  \
    exit(errors::NavError::Initalize::Configuration::NoConfigurationFile); \
  }

#define CHECK_PARSE_ERR(parse_result)                                                                     \
  if (parse_result.error().has_value()) {                                                                 \
    nav_error("Parse configuration file \"{}\" failed ", std::filesystem::absolute(config_path).c_str()); \
    exit(errors::NavError::Initalize::Configuration::ParseConfigurationError);                            \
  }
namespace navp::solution {

Config::Config() = default;
Config::~Config() = default;

SppConfig::SppConfig() = default;

SppConfig::~SppConfig() = default;

void SppConfig::from(const char* cfg_path) {
  CHECK_FILE_EXIST(cfg_path);
  auto _config = rfl::json::load<SppConfigContent>(cfg_path);
  cfg_ = std::move(_config.value());
}

const Config* SppConfig::get() { return static_cast<const Config*>(this); }

}  // namespace navp::solution