#pragma once

#include <spdlog/logger.h>

#include "algorithm/algorithm.hpp"
#include "sensors/gnss/enums.hpp"
#include "solution/solution.hpp"
#include "toml++/toml.hpp"
#include "utils/null_mutex.hpp"

namespace navp {
class GlobalConfig;
}

// forward declaration
namespace navp::sensors::gnss {
class GnssHandler;
class GnssNavRecord;
class EphemerisSolver;
class GnssPayload;
}  // namespace navp::sensors::gnss

namespace navp::solution {

/*
 * Register cofiguration parse exception
 */
REGISTER_NAV_RUNTIME_ERROR_CHILD(ConfigParseError, NavRuntimeError);

using sensors::gnss::ConstellationEnum;
using sensors::gnss::EphemerisSolver;
using sensors::gnss::GnssNavRecord;
using sensors::gnss::IonoModelEnum;
using sensors::gnss::ObsCodeEnum;
using sensors::gnss::RandomModelEnum;
using sensors::gnss::TropModelEnum;

class NAVP_EXPORT NavConfigManger : public toml::parse_result {
 public:
  NavConfigManger() = default;
  NavConfigManger(std::string_view cfg_path);
  ~NavConfigManger() noexcept = default;

  NavConfigManger(const NavConfigManger&) noexcept = delete;
  NavConfigManger& operator=(const NavConfigManger&) noexcept = delete;

  NavConfigManger(NavConfigManger&&) noexcept = default;
  NavConfigManger& operator=(NavConfigManger&&) noexcept = default;

  using toml::parse_result::operator[];

  friend std::ostream& operator<<(std::ostream& os, const NavConfigManger& cfg) {
    os << static_cast<toml::parse_result>(cfg);
    return os;
  }

  // return config path
  NAV_NODISCARD_UNUNSED auto path() const noexcept -> std::string_view;

  // solution mode
  NAV_NODISCARD_ERROR_HANDLE auto solution_mode() const noexcept -> SolutionModeEnum;

  // algorithm
  NAV_NODISCARD_ERROR_HANDLE auto algorithm() const noexcept -> algorithm::AlgorithmEnum;

  // task name
  NAV_NODISCARD_ERROR_HANDLE auto task_name() const noexcept -> std::string;

  // project name
  NAV_NODISCARD_ERROR_HANDLE auto proj_name() const noexcept -> std::string;

  // executor name
  NAV_NODISCARD_ERROR_HANDLE auto executor_name() const noexcept -> std::string;

  // executor time
  NAV_NODISCARD_ERROR_HANDLE auto executor_time() const noexcept -> EpochUtc;

  // solution base station
  NAV_NODISCARD_ERROR_HANDLE auto base_station(bool enabled_mt = false) const noexcept
      -> std::shared_ptr<sensors::gnss::GnssHandler>;

  // solution rover station
  NAV_NODISCARD_ERROR_HANDLE auto rover_station(bool enabled_mt = false) const noexcept
      -> std::shared_ptr<sensors::gnss::GnssHandler>;

  // logger
  NAV_NODISCARD_ERROR_HANDLE auto logger() const noexcept -> std::shared_ptr<spdlog::logger>;

  // capacity
  NAV_NODISCARD_ERROR_HANDLE auto capacity() const noexcept -> i32;

  // filters
  NAV_NODISCARD_ERROR_HANDLE auto filters() const noexcept -> std::vector<std::string_view>;

  // ratio limit
  NAV_NODISCARD_ERROR_HANDLE auto ratio() const noexcept -> f32;

  // output stream
  NAV_NODISCARD_ERROR_HANDLE auto output_dir() const noexcept -> std::string;
};

}  // namespace navp::solution

template <>
struct std::formatter<navp::solution::NavConfigManger> : std::formatter<std::string> {
  auto format(const navp::solution::NavConfigManger& obj, auto& ctx) const {
    std::ostringstream oss;
    oss << obj;
    return std::formatter<std::string>::format(oss.str(), ctx);
  }
};

namespace navp {

class NAVP_EXPORT GlobalConfig {
 public:
  static void initialize(std::string config_path) noexcept;

  static std::shared_ptr<sensors::gnss::GnssHandler> get_station_st(std::string_view station_name) noexcept;

  static std::shared_ptr<sensors::gnss::GnssHandler> get_station_mt(std::string_view station_name) noexcept;

  static std::shared_ptr<spdlog::logger> get_logger(std::string_view logger_name) noexcept;

 private:
  GlobalConfig();

  static solution::NavConfigManger& _get_instance() noexcept;

  static std::unique_ptr<sensors::gnss::GnssPayload> _get_station_handler(std::string_view station_name) noexcept;

  static std::shared_ptr<spdlog::logger> _get_logger(std::string_view logger_name) noexcept;

  static std::string config_path_;
  static std::once_flag flag_;
  static solution::NavConfigManger config_;
  static std::mutex mutex_;

  // map of station handler
  static std::unordered_map<std::string, std::shared_ptr<sensors::gnss::GnssHandler>> st_station_handler_map_;
  static std::unordered_map<std::string, std::shared_ptr<sensors::gnss::GnssHandler>> mt_station_handler_map_;
};

}  // namespace navp

template <>
struct std::formatter<toml::source_region> : std::formatter<std::string> {
  auto format(const toml::source_region& obj, auto& ctx) const {
    std::ostringstream oss;
    oss << obj;
    return std::formatter<std::string>::format(oss.str(), ctx);
  }
};