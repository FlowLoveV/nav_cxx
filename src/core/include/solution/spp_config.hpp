#pragma once

#include <rfl.hpp>

#include "solution/config.hpp"
#include "utils/types.hpp"

namespace navp::solution {

// config elements
typedef rfl::Rename<"navigation_files_path", std::vector<std::string>> NavigationPath;
typedef rfl::Rename<"observation_path", std::string> ObservationPath;
typedef rfl::Rename<"output_path", std::string> OutputPath;

typedef rfl::Rename<"minimum_elevation", f64> ElevationConfig;
typedef rfl::Rename<"minimum_snr", f64> SnrConfig;

typedef rfl::Rename<"filters", std::vector<std::string>> FilterConfig;

struct NAVP_EXPORT SppConfigContent {
  NavigationPath nav_path;
  ObservationPath obs_path;
  OutputPath out_path;
  ElevationConfig min_e;
  SnrConfig min_s;
  FilterConfig filter;
};

class NAVP_EXPORT SppConfig : public Config {
 public:
  SppConfig();
  virtual ~SppConfig() override;

  virtual void from(const char* cfg_path) override;

  virtual const Config* get() override;

 protected:
  SppConfigContent cfg_;
};

}  // namespace navp::solution