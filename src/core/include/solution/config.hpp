#pragma once

#include "utils/macro.hpp"
namespace navp::solution {

namespace details {
#define REGISTER_CONFIG_ITEM(name, id) inline constexpr auto name = id;
}  // namespace details

// project configuration
REGISTER_CONFIG_ITEM(ProjCfg, "meta");
REGISTER_CONFIG_ITEM(TaskName, "task");
REGISTER_CONFIG_ITEM(ProjectName, "project");
REGISTER_CONFIG_ITEM(ExecuteTime, "time");
REGISTER_CONFIG_ITEM(Executor, "executor");

// io config
REGISTER_CONFIG_ITEM(IoCfg, "io");
REGISTER_CONFIG_ITEM(NavPath, "nav_path");
REGISTER_CONFIG_ITEM(ObsPath, "obs_path");
REGISTER_CONFIG_ITEM(OutputPath, "out_path");
REGISTER_CONFIG_ITEM(ReferencePath, "ref_path");

// model config
REGISTER_CONFIG_ITEM(TropModel, "trop");
REGISTER_CONFIG_ITEM(IonoModel, "iono");

// complex filter
REGISTER_CONFIG_ITEM(FilterCfg, "filter")

}  // namespace navp::solution