#pragma once

#include <deque>

#include "sensors/gnss/gnss_handler.hpp"
#include "solution/solution.hpp"
#include "solution/task.hpp"

namespace navp::solution {

using sensors::gnss::GnssStationHandler;

class Rtk;

class NAVP_EXPORT Rtk : public ConfigTask {
 public:
  Rtk(std::string_view cfg_path) noexcept;

 protected:
  GnssStationHandler rover_, base_;           ///> rover server and base server
  std::deque<PvtSolutionRecord> sol_;  ///> solution
};

}  // namespace navp::solution