#pragma once

#include "io/stream.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "solution/solution.hpp"
#include "solution/task.hpp"

namespace navp::sensors::gnss {

// forward declaration
class BrdcEphSolver;

}  // namespace navp::sensors::gnss

namespace navp::solution {

using navp::sensors::gnss::BrdcEphSolver;
using navp::sensors::gnss::GnssNavRecord;
using navp::sensors::gnss::GnssObsRecord;

class NAVP_EXPORT Spp : public ConfigTask {
 public:
  Spp(std::string_view cfg_path) noexcept;
  virtual ~Spp() override;
  virtual void solve() override;

 protected:
  void init();

  std::vector<GnssNavRecord> nav_;             /// record of gnss navigation
  GnssObsRecord obs_;                          /// record of gnss observation
  std::unique_ptr<BrdcEphSolver> eph_solver_;  /// broadcast ephemeris solver
  std::deque<PvtSolution> sol_;                /// solution
  std::unique_ptr<io::Stream> obs_stream_;     /// obs stream
  EpochUtc t_;                                 /// record of runtime Epoch
};

}  // namespace navp::solution