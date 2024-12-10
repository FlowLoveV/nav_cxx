#pragma once

#include <deque>

#include "solution/gnss_handler.hpp"
#include "solution/solution.hpp"
#include "solution/task.hpp"

namespace navp::solution {

class NAVP_EXPORT Spp : public ConfigTask {
 public:
  Spp(std::string_view cfg_path) noexcept;
  virtual ~Spp() override;

 protected:
  std::unique_ptr<GnssHandler> rover_;  ///> rover server
  std::deque<PvtSolutionRecord> sol_;   ///> solution
};

class NAVP_EXPORT FgoSpp : public Spp {
 public:
  using Spp::Spp;
  virtual void solve() override;
  virtual ~FgoSpp() override;
};

}  // namespace navp::solution