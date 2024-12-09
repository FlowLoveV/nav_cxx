#include "solution/rtk.hpp"

#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::solution {

Rtk::Rtk(std::string_view cfg_path) noexcept : ConfigTask(cfg_path) {
  rover_.initialize(config_, true);
  rover_.initialize(config_, false);
}

Rtk::~Rtk() = default;

void Rtk::solve() {
  //
}

}  // namespace navp::solution