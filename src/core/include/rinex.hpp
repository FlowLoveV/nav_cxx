#pragma once

#include "types.hpp"

namespace navp::rinex {

using std::string;
using std::string_view;

// rinex version
struct RinexVersion {
  u8 major, minor;
};

// rinex file type
enum class RinexType : u8 {
  Observation,
  Navigation,
  Meteo,
  Clock,
  Ionosphere,
  Antenna,
  DORIS,
};

// rinex leap information
struct Leap {
  // Current Number of leap seconds since 6 Jan 1980
  u32 leap;
  /// ΔtLS : "future or past leap second(s)",
  /// actual number of leap seconds between GPS/GAL and GLO,
  /// or BDS and UTC.
  std::optional<u32> delta_tls;
  /// weeks counter
  std::optional<u32> week;
  /// days counter
  std::optional<u32> days;
};

// rinex observation header
class ObservationHeader {
 public:
};

// rinex header
// only record some import information
class RinexHeader {
 public:
  RinexVersion version;
  RinexType type;
  std::optional<ConstellationEnum> constellation;
  std::optional<Leap> leap;
};

}  // namespace navp::rinex