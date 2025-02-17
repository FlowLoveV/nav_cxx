#include "io/custom/solution_stream.hpp"

#include "solution/solution.hpp"
#include "utils/angle.hpp"
#include "utils/space.hpp"

namespace navp::io::custom {

void encode_time(SolutionStream& stream, const EpochUtc& time) noexcept {
  switch (stream.format_options().time_type) {
    case TimeTypeEnum::UTC: {
      stream << std::format("{}", time);
      break;
    }
    case TimeTypeEnum::GPS: {
      stream << std::format("{}", time.gps_time<GpsClock>());
      break;
    }
    case TimeTypeEnum::BDT: {
      stream << std::format("{}", time.gps_time<BdsClock>());
      break;
    }
    default: {
      stream.error("Unsupported time type, the TimeTypeEnum should be 0(UTC), 1(GPST), 2(BDST), {} is invalid",
                   (u8)stream.format_options().time_type);
    }
  }
}

void encode_angle(SolutionStream& stream, f64 angle) noexcept {
  switch (stream.format_options().angle_type) {
    case AngleTypeEnum::DMS: {
      auto dms = DDmmss<f64>::from_radians(angle);
      if (dms.negative)
        stream << "-";
      else
        stream << " ";
      stream << std::format("{:>03}:{:>02}:{:2.6f}", dms.hh, dms.mm, dms.ss);
      break;
    }
    case AngleTypeEnum::DEG: {
      stream << std::format("{:>5.8f}", to_degress(angle));
      break;
    }
    case AngleTypeEnum::RAD: {
      stream << std::format("{:>3.15f}", angle);
      break;
    }
    default: {
      stream.error("Unsupported angle type, the AngleTypeEnum should be 0(DMS), 1(DEG), 2(RAD), {} is invalid",
                   (u8)stream.format_options().angle_type);
    }
  }
}

void encode_coordinate(SolutionStream& stream, const utils::coordinate_t& coordinate, char separator) noexcept {
  switch (stream.format_options().coordinate_type) {
    case CoordinateTypeEnum::XYZ: {
      stream << std::format("{:>14.4f}{}{:>14.4f}{}{:>14.4f}", coordinate.x(), separator, coordinate.y(), separator,
                            coordinate.z());
      break;
    }
    case CoordinateTypeEnum::BLH: {
      encode_angle(stream, coordinate.x());
      stream << separator;
      encode_angle(stream, coordinate.y());
      stream << separator;
      stream << std::format("{:>7.4f}", coordinate.z());
      break;
    }
    case CoordinateTypeEnum::ENU: {
      stream << std::format("{:>6.4f}{}{:>6.4f}{}{:>6.4f}", coordinate.x(), separator, coordinate.y(), separator,
                            coordinate.z());
      break;
    }
    default: {
      stream.error(
          "Unsupported coordinate type, the CoordinateTypeEnum should be 0(XYZ), 1(BLH), 2(ENU), {} is invalid",
          (u8)stream.format_options().coordinate_type);
    }
  }
}

void encode_velocity(SolutionStream& stream, const utils::coordinate_t& coordinate, char separator) noexcept {
  switch (stream.format_options().coordinate_type) {
    case CoordinateTypeEnum::XYZ:  // break through
    case CoordinateTypeEnum::ENU: {
      stream << std::format("{:>10.4f}{}{:>10.4f}{}{:>10.4f}", coordinate.x(), separator, coordinate.y(), separator,
                            coordinate.z());
      break;
    }
    default: {
      stream.error("Unsupported velocity type, the CoordinateTypeEnum should be 0(XYZ), 1(ENU), {} is invalid",
                   (u8)stream.format_options().coordinate_type);
    }
  }
}

// todo
void encode_solution_info(SolutionStream& stream, const solution::PvtSolutionRecord& pvt_record,
                          char separator) noexcept {
  stream << std::format("{:1d}", static_cast<u8>(pvt_record.mode)) << separator;
  stream << std::format("{:02d}", pvt_record.ns);
}

void encode_pvt_record_header(SolutionStream& stream) {}

void encode_pvt_record(SolutionStream& stream, const solution::PvtSolutionRecord& record) {
  const auto& format_options = stream.format_options();
  auto separator = format_options.separator;

  encode_time(stream, record.time);  // epoch
  stream << separator;

  // position
  switch (format_options.coordinate_type) {
    case CoordinateTypeEnum::XYZ: {
      encode_coordinate(stream, record.position, separator);
      break;
    }
    case CoordinateTypeEnum::BLH: {
      encode_coordinate(stream, record.blh, separator);
      break;
    }
    default: {
      stream.error("Unsupported coordinate type, the CoordinateTypeEnum should be 0(XYZ), 1(BLH), {} is invalid",
                   (u8)format_options.coordinate_type);
    }
  }
  stream << separator;

  // velocity
  encode_velocity(stream, record.velocity, separator);
  stream << separator;

  // solution information
  encode_solution_info(stream, record, separator);
}

// todo
void SolutionStream::decode_record(Record& record) { nav_error("Not implemented"); }

void SolutionStream::encode_record(const Record& record) {
  if (auto pvt_record_ptr = dynamic_cast<const solution::PvtSolutionRecord*>(&record)) {
    encode_pvt_record(*this, *pvt_record_ptr);
    new_line();
  }
}

}  // namespace navp::io::custom