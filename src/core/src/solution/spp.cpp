#include "solution/spp.hpp"

#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "solution/config.hpp"

using navp::sensors::gnss::Sv;

namespace navp::solution {

Spp::Spp(std::string_view cfg_path) noexcept : ConfigTask(cfg_path) { rover_.initialize(config_); }

Spp::~Spp() = default;

void Spp::solve() {
  while (rover_.get_next_observation()) {
    PvtSolutionRecord sol;
    

    // solve position
    sol.mode = SolutionModeEnum::SINGLE;
    sol_.emplace_back(sol);
  }
}

}  // namespace navp::solution