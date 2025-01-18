#pragma once

#include <deque>

#include "sensors/gnss/gnss.hpp"
#include "solution/solution.hpp"
#include "solution/task.hpp"

namespace navp::solution {

using sensors::gnss::GnssHandler;

class Rtk;

class NAVP_EXPORT Rtk : public Task {
 public:
 Rtk(std::string_view cfg_path,bool enabled_mt = false);

  virtual ~Rtk() override = default;

 protected:
  std::shared_ptr<GnssHandler> rover_, base_;  ///> rover server and base server
  std::deque<PvtSolutionRecord> sol_;          ///> solution
};

}  // namespace navp::solution