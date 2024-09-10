#pragma once

#include <rfl.hpp>
#include <any>

#include "utils/types.hpp"

namespace navp::solution {

struct SppConfig {
  rfl::Rename<"navigation_files_path", std::vector<std::string>> nav_path;
  rfl::Rename<"observation_path", std::string> obs_path;
  rfl::Rename<"output_path", std::string> out_path;
  rfl::Rename<"minimum_elevation", f64> min_e;
  rfl::Rename<"minimum_snr", f64> min_s;
  rfl::Rename<"filters", std::vector<std::string>> filter; 
};

}  // namespace navp::solution