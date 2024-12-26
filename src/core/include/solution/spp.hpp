#pragma once

#include "sensors/gnss/gnss.hpp"
#include "solution/solution.hpp"
#include "utils/option.hpp"

namespace navp::solution {

using sensors::gnss::ClockParameterMap;
using sensors::gnss::CodeMap;
using sensors::gnss::GnssHandler;

class NAVP_EXPORT Spp {
 public:
  Spp(std::shared_ptr<GnssHandler> handler) noexcept;
  std::shared_ptr<GnssHandler> gnss_handler() noexcept;

  bool next_solution() noexcept;

  auto solution() const noexcept -> const PvtSolutionRecord*;

  ~Spp() = default;

 protected:
  void update_clock_map(const CodeMap& code_map) const noexcept;
  std::unique_ptr<PvtSolutionRecord> sol_;  // solution
  std::shared_ptr<GnssHandler> rover_;      // rover handler
  mutable ClockParameterMap clock_map_;     // clock parameter map
};

class NAVP_EXPORT FgoSpp : public Spp {
 public:
  using Spp::Spp;
};

}  // namespace navp::solution