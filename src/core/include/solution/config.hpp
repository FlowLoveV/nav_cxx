#pragma once

#include <unordered_set>

#include "sensors/gnss/enums.hpp"
#include "solution/solution.hpp"
#include "toml++/toml.hpp"

// forward declaration
namespace navp::sensors::gnss {
class GnssNavRecord;
class EphemerisSolver;
}  // namespace navp::sensors::gnss

namespace navp::solution {

#define REGISTER_CONFIG_ITEM(name, id) inline constexpr auto name = id;
// project configuration
REGISTER_CONFIG_ITEM(ProjCfg, "meta");
REGISTER_CONFIG_ITEM(TaskNameCfg, "task");        // std::string
REGISTER_CONFIG_ITEM(ProjectNameCfg, "project");  // std::string
REGISTER_CONFIG_ITEM(ExecuteTimeCfg, "time");     // std::string
REGISTER_CONFIG_ITEM(ExecutorCfg, "executor");    // std::string

// io config
REGISTER_CONFIG_ITEM(IoCfg, "io");
REGISTER_CONFIG_ITEM(RoverNavPathCfg, "rover_nav_path");           // std::string
REGISTER_CONFIG_ITEM(BaseNavPathCfg, "base_nav_path");             // std::string
REGISTER_CONFIG_ITEM(RoverObsPathCfg, "rover_obs_path");           // std::string
REGISTER_CONFIG_ITEM(BaseObsPathCfg, "base_obs_path");             // std::string
REGISTER_CONFIG_ITEM(OutputPathCfg, "out_path");                   // std::string
REGISTER_CONFIG_ITEM(ReferencePathCfg, "ref_path");                // std::string
REGISTER_CONFIG_ITEM(RoverRefPosStyleCfg, "rover_ref_pos_style");  // 0-XYZ 1-BLH 2-ENU
REGISTER_CONFIG_ITEM(RoverRefPosCfg, "rover_ref_pos");             // array[f64]
REGISTER_CONFIG_ITEM(BaseRefPosStyleCfg, "base_ref_pos_style");    // 0-XYZ 1-BLH 2-ENU
REGISTER_CONFIG_ITEM(BaseRefPosCfg, "base_ref_pos");               // array[f64]

// model config
REGISTER_CONFIG_ITEM(ModelCfg, "model")
REGISTER_CONFIG_ITEM(EnabledCodeCfg, "enabled_code");    // table 
REGISTER_CONFIG_ITEM(TropModelCfg, "trop");              // integer (u8)
REGISTER_CONFIG_ITEM(IonoModelCfg, "iono");              // integer (u8)
REGISTER_CONFIG_ITEM(SolutionModeCfg, "solution_mode");  // integer (u8)

// complex filter
REGISTER_CONFIG_ITEM(FilterCfg, "filter")  // std::string

#undef REGISTER_CONFIG_ITEM

/*
 * Register cofiguration parse exception
 */
REGISTER_NAV_RUNTIME_ERROR_CHILD(ConfigParseError, NavRuntimeError);

template <typename T>
using ConfigResult = Result<T, ConfigParseError>;

using sensors::gnss::ConstellationEnum;
using sensors::gnss::EphemerisSolver;
using sensors::gnss::GnssNavRecord;
using sensors::gnss::IonoModelEnum;
using sensors::gnss::ObsCodeEnum;
using sensors::gnss::RandomModelEnum;
using sensors::gnss::TropModelEnum;

class NAVP_EXPORT NavConfigManger : public toml::parse_result {
 public:
  using CodeTypeMap = std::unordered_map<ConstellationEnum, std::unordered_set<ObsCodeEnum>>;

  NavConfigManger(std::string_view cfg_path);
  ~NavConfigManger() noexcept;

  using toml::parse_result::operator[];

  friend std::ostream& operator<<(std::ostream& os, const NavConfigManger& cfg) {
    os << static_cast<toml::parse_result>(cfg);
    return os;
  }

  // return config path
  NAV_NODISCARD_UNUNSED auto path() const noexcept -> std::string_view;

  // return rover observation stream
  NAV_NODISCARD_ERROR_HANDLE auto rover_obs() const noexcept -> ConfigResult<std::unique_ptr<io::Stream>>;

  // return rover observation stream
  NAV_NODISCARD_ERROR_HANDLE auto base_obs() const noexcept -> ConfigResult<std::unique_ptr<io::Stream>>;

  // return rover GnssNavRecord vector
  NAV_NODISCARD_ERROR_HANDLE auto rover_nav() const noexcept -> ConfigResult<std::vector<GnssNavRecord>>;

  // return base GnssNavRecord vector
  NAV_NODISCARD_ERROR_HANDLE auto base_nav() const noexcept -> ConfigResult<std::vector<GnssNavRecord>>;

  // rover reference position
  NAV_NODISCARD_ERROR_HANDLE auto rover_ref_pos(
      utils::CoordSystemEnum style = utils::CoordSystemEnum::XYZ) const noexcept -> ConfigResult<utils::NavVector3f64>;

  // base reference position
  NAV_NODISCARD_ERROR_HANDLE auto base_ref_pos(
      utils::CoordSystemEnum style = utils::CoordSystemEnum::XYZ) const noexcept -> ConfigResult<utils::NavVector3f64>;

  // enabled obs code
  NAV_NODISCARD_ERROR_HANDLE auto enabled_code() const noexcept -> ConfigResult<CodeTypeMap>;

  // trop model
  NAV_NODISCARD_ERROR_HANDLE auto trop_model() const noexcept -> ConfigResult<TropModelEnum>;

  // iono model
  NAV_NODISCARD_ERROR_HANDLE auto iono_model() const noexcept -> ConfigResult<IonoModelEnum>;

  // random model
  NAV_NODISCARD_ERROR_HANDLE auto random_model() const noexcept -> ConfigResult<RandomModelEnum>;

  // solution mode
  NAV_NODISCARD_ERROR_HANDLE auto solution_mode() const noexcept -> ConfigResult<SolutionModeEnum>;
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

template <>
struct std::formatter<toml::source_region> : std::formatter<std::string> {
  auto format(const toml::source_region& obj, auto& ctx) const {
    std::ostringstream oss;
    oss << obj;
    return std::formatter<std::string>::format(oss.str(), ctx);
  }
};