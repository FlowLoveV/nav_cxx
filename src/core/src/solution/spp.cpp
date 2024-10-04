
#include "solution/spp.hpp"

#include <filesystem>

#include "io/rinex/rinex.hpp"
#include "rfl/json.hpp"
#include "utils/error.hpp"
#include "utils/logger.hpp"

namespace navp::solution {

Spp::Spp(const char* config_path) noexcept {
  // read configuration
  if (!std::filesystem::exists(config_path)) {
    nav_error("No Spp configuration file found at {}", config_path);
    exit(errors::NavError::Initalize::Configuration::NoSppConfigurationFile);
  }
  auto config = rfl::json::load<SppConfig>(config_path);
  if (config.error().has_value()) {
    nav_error("Parse Spp configuration file \"{}\" failed ", std::filesystem::absolute(config_path).c_str());
    exit(errors::NavError::Initalize::Configuration::ParseSppConfigurationError);
  }
  this->spp.config = std::move(config.value_or({}));
  // current read from rinex

  // read navigition
  read_rinex_nav();
  // read observation
  read_rinex_obs();
}

Spp::Spp(std::istream& config_stream) noexcept {
  // read configuration
  auto config = rfl::json::read<SppConfig>(config_stream);
  if (config.error().has_value()) {
    nav_error("Parse Spp configuration file stream failed ");
    exit(errors::NavError::Initalize::Configuration::ParseSppConfigurationError);
  }
  this->spp.config = std::move(config.value_or({}));
  // current read from rinex

  // read navigition
  read_rinex_nav();
  // read observation
  read_rinex_obs();
}

void Spp::read_rinex_nav() noexcept {
  auto& nav_path_vec = this->spp.config.nav_path.get();
  for (const auto& path : nav_path_vec) {
    auto nav = io::rinex::RinexNav(path.c_str());
    this->spp.nav.emplace_front(std::move(nav.nav));
  }
}

void Spp::read_rinex_obs() noexcept {
  auto& obs_path = this->spp.config.obs_path.get();
  auto obs = io::rinex::RinexObs(obs_path.c_str());
  this->spp.obs.obs_list = std::move(obs.obs_list);
}

const std::forward_list<RecordGnssNav>& Spp::get_navigation() const noexcept { return this->spp.nav; }

const RecordGnssObs Spp::get_observation() const noexcept { return this->spp.obs; }

std::forward_list<PositioningResult>& Spp::get_result() const noexcept { return this->spp.result; }

}  // namespace navp::solution