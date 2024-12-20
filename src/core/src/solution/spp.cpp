#include "solution/spp.hpp"

#include "sensors/gnss/atmosphere.hpp"
#include "sensors/gnss/ephemeris_solver.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "solution/config.hpp"

namespace navp::solution {

using sensors::gnss::Sv;

Spp::Spp(std::shared_ptr<GnssStationHandler> handler) noexcept : rover_(handler) {}

std::shared_ptr<GnssStationHandler> Spp::gnss_handler() noexcept { return rover_; }

PvtSolutionRecord Spp::next() noexcept {
  PvtSolutionRecord sol;
  // auto obs = rover_->load_next_observation();
  // auto sv_map = rover_->solve_satellites_info();
  // auto handler = rover_->generate_rawobs_handler();

  // sol.time = rover_->latest_epoch();  // set time

  // auto size = sv_vector.size();

  // utils::NavVectorDf64 parameter(3 + clock_map.size());
  // utils::NavVectorDf64 correction(3 + clock_map.size());
  // utils::NavVectorDf64 residual(size);
  // utils::NavMatrixDf64 jacobian(size, 3 + size);
  // utils::NavMatrixDf64 weight(size, size);

SppLoop: {}

  return sol;
}

}  // namespace navp::solution