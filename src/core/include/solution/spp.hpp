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
  virtual void solve() override;

 protected:
  GnssHandler rover_;                   ///> rover server
  std::deque<PvtSolutionRecord> sol_;  ///> solution
};

}  // namespace navp::solution