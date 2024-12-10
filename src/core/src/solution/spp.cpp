#include "solution/spp.hpp"

#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "solution/config.hpp"

namespace navp::solution {

Spp::Spp(std::string_view cfg_path) noexcept : ConfigTask(cfg_path), rover_(std::make_unique<GnssHandler>()) {
  rover_->initialize(config_);
}

Spp::~Spp() = default;

void FgoSpp::solve() {
  while (rover_->load_next_observation()) {
    auto available_satellite = rover_->available_satellites();

  }
}

FgoSpp::~FgoSpp() = default;

}  // namespace navp::solution