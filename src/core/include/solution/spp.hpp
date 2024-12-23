#pragma once

#include "sensors/gnss/gnss.hpp"
#include "solution/solution.hpp"
#include "utils/option.hpp"

namespace navp::solution {

using sensors::gnss::GnssHandler;

class NAVP_EXPORT Spp {
 public:
  Spp(std::shared_ptr<GnssHandler> handler) noexcept;
  std::shared_ptr<GnssHandler> gnss_handler() noexcept;

  Option<PvtSolutionRecord> next_solution() noexcept;

  ~Spp() = default;

 protected:
  std::shared_ptr<GnssHandler> rover_;       ///> rover handler
};

class NAVP_EXPORT FgoSpp : public Spp {
 public:
  using Spp::Spp;
};

}  // namespace navp::solution