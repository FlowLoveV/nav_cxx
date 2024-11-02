#pragma once

#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "solution/mode.hpp"
#include "solution/spp_config.hpp"
#include "utils/macro.hpp"

namespace navp::solution {

using navp::sensors::gnss::GnssNavRecord;
using navp::sensors::gnss::GnssObsRecord;

class NAVP_EXPORT Spp {
 public:
  Spp(const char* config_path) noexcept;
  Spp(std::istream& config_stream) noexcept;

 private:
};

}  // namespace navp::solution