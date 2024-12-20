#pragma once

#include "sensors/gnss/gnss_handler.hpp"
#include "solution/solution.hpp"

namespace navp::solution {

using sensors::gnss::GnssStationHandler;

class NAVP_EXPORT Spp {
 public:
  Spp(std::shared_ptr<GnssStationHandler> handler) noexcept;
  std::shared_ptr<GnssStationHandler> gnss_handler() noexcept;

  PvtSolutionRecord next() noexcept;

  ~Spp() = default;

 protected:
  std::shared_ptr<GnssStationHandler> rover_;       ///> rover handler
};

class NAVP_EXPORT FgoSpp : public Spp {
 public:
  using Spp::Spp;
};

}  // namespace navp::solution