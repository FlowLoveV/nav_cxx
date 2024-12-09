#pragma once

#include <deque>

#include "solution/gnss_handler.hpp"
#include "solution/solution.hpp"
#include "solution/task.hpp"

namespace navp::solution {

class Rtk;

class NAVP_EXPORT Rtk : public ConfigTask {
 public:
  Rtk(std::string_view cfg_path) noexcept;

  virtual void solve() override;

  virtual ~Rtk() override;

 protected:
  GnssHandler rover_, base_;           ///> rover server and base server
  std::deque<PvtSolutionRecord> sol_;  ///> solution
};

}  // namespace navp::solution