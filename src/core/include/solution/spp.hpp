#pragma once

#include "mode.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "spp_config.hpp"
namespace navp::solution {

using navp::sensors::gnss::RecordGnssNav;
using navp::sensors::gnss::RecordGnssObs;

struct SolutionSpp {
  SppConfig config;                                     /// spp solution config
  std::forward_list<RecordGnssNav> nav;                 /// spp navigation list
  RecordGnssObs obs;                                    /// spp observation
  mutable std::forward_list<PositioningResult> result;  /// spp result list
};

class Spp {
 public:
  Spp() = default;
  ~Spp() = default;
  Spp(const char* config_path) noexcept;
  Spp(std::istream& config_stream) noexcept;

  // get navigation data
  const std::forward_list<RecordGnssNav>& get_navigation() const noexcept;
  // get observation date
  const RecordGnssObs get_observation() const noexcept;
  // get result date
  std::forward_list<PositioningResult>& get_result() const noexcept;

 protected:
  void read_rinex_nav() noexcept;
  void read_rinex_obs() noexcept;

 private:
  SolutionSpp spp;
};

}  // namespace navp::solution