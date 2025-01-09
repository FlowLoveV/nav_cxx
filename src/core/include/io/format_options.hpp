#pragma once

#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace navp::io {

enum class AngleTypeEnum : u8 {
  DMS = 0,
  DEG = 1,
  RAD = 2,
};

enum class TimeTypeEnum : u8 {
  UTC = 0,
  GPS = 1,
  BDT = 2,
};

enum class CoordinateTypeEnum : u8 {
  XYZ = 0,
  BLH = 1,
  ENU = 2,
};

struct NAVP_EXPORT FormatOptions {
  char separator = ' ';
  bool show_details = false;
  AngleTypeEnum angle_type = AngleTypeEnum::DMS;
  TimeTypeEnum time_type = TimeTypeEnum::UTC;
  CoordinateTypeEnum coordinate_type = CoordinateTypeEnum::XYZ;
};

}  // namespace navp::io