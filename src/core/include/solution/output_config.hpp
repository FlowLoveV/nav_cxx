#pragma once

#include "utils/angle.hpp"
#include "utils/space.hpp"
#include "utils/time.hpp"

namespace navp::solution {

namespace details {

struct TimeConfig {
  TimeScaleEnum time_type;
  std::string date_format;
  u8 precision;
};

struct CoordConfig {
  utils::CoordSystemEnum coord_type;
  u8 precision;
};

struct AngleConfig {
  AngleEnum angle_type;
  u8 precision;
};

struct VarianceConfig {};

}  // namespace details

struct OutputConfig {
  details::TimeConfig time_config;
  details::CoordConfig coord_config;
  details::AngleConfig angle_config;
};

void output(std::istream& os);

}  // namespace navp::solution