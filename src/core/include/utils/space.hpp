#pragma once

#include <span>
#include <vector>

#include "utils/eigen.hpp"
#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace navp::utils {
using coordinate_t = NavVector3f64;

enum class NAVP_EXPORT CoordSystemEnum : u8 {
  XYZ,
  BLH,
  ENU,
};

template <CoordSystemEnum T>
class Coordinate;

namespace details {
struct CoordinatePayload : public coordinate_t {
  using coordinate_t::Matrix;

  coordinate_t& coord() & { return *static_cast<coordinate_t*>(this); }
  const coordinate_t& coord() const& { return *static_cast<const coordinate_t*>(this); }
  coordinate_t&& coord() && { return std::move(*static_cast<coordinate_t*>(this)); }
  const coordinate_t&& coord() const&& { return std::move(*static_cast<const coordinate_t*>(this)); }

  using coordinate_t::operator-;
  using coordinate_t::operator+;
  using coordinate_t::operator+=;
  using coordinate_t::operator-=;
  using coordinate_t::operator==;

  template <CoordSystemEnum T>
  coordinate_t operator-(const Coordinate<T>& rhs) const noexcept {
    return this->coord() - rhs.coord();
  }
};

void ecef2pos(const f64* r, f64* pos);
void xyz2enu(const f64* pos, f64* E);
void pos2ecef(const f64* pos, f64* r);

}  // namespace details

template <>
class NAVP_EXPORT Coordinate<CoordSystemEnum::XYZ> : public details::CoordinatePayload {
 public:
  using details::CoordinatePayload::CoordinatePayload;
  Coordinate(const Coordinate&) = default;
  Coordinate(Coordinate&&) = default;
  Coordinate& operator=(const Coordinate&) = default;
  Coordinate& operator=(Coordinate&&) = default;

  auto to_blh() const noexcept -> Coordinate<CoordSystemEnum::BLH>;
  auto to_enu_matrix() const noexcept -> NavMatrix33f64;
  auto to_enu(const Coordinate&) const noexcept -> Coordinate<CoordSystemEnum::ENU>;
  auto to_enu(const std::span<Coordinate>&) const noexcept -> std::vector<Coordinate<CoordSystemEnum::ENU>>;
  auto to_enu(const Coordinate<CoordSystemEnum::BLH>&) const noexcept -> Coordinate<CoordSystemEnum::ENU>;
  auto to_enu(const std::span<Coordinate<CoordSystemEnum::BLH>>&) const noexcept
      -> std::vector<Coordinate<CoordSystemEnum::ENU>>;

  // convariance
  auto conv_enu(const NavMatrix33f64&) const -> NavMatrix33f64;
};

template <>
class NAVP_EXPORT Coordinate<CoordSystemEnum::BLH> : public details::CoordinatePayload {
 public:
  using details::CoordinatePayload::CoordinatePayload;
  Coordinate(const Coordinate&) = default;
  Coordinate(Coordinate&&) = default;
  Coordinate& operator=(const Coordinate&) = default;
  Coordinate& operator=(Coordinate&&) = default;

  auto to_xyz() const noexcept -> Coordinate<CoordSystemEnum::XYZ>;
  auto to_enu_matrix() const noexcept -> NavMatrix33f64;
  auto to_enu(const Coordinate&) const noexcept -> Coordinate<CoordSystemEnum::ENU>;
  auto to_enu(const std::span<Coordinate>&) const noexcept -> std::vector<Coordinate<CoordSystemEnum::ENU>>;
  auto to_enu(const Coordinate<CoordSystemEnum::XYZ>&) const noexcept -> Coordinate<CoordSystemEnum::ENU>;
  auto to_enu(const std::span<Coordinate<CoordSystemEnum::XYZ>>&) const noexcept
      -> std::vector<Coordinate<CoordSystemEnum::ENU>>;

  // convariance
  auto conv_enu(const NavMatrix33f64&) const noexcept -> NavMatrix33f64;
};

template <>
class NAVP_EXPORT Coordinate<CoordSystemEnum::ENU> : public details::CoordinatePayload {
 public:
  using details::CoordinatePayload::CoordinatePayload;
  Coordinate(const Coordinate&) = default;
  Coordinate(Coordinate&&) = default;
  Coordinate& operator=(const Coordinate&) = default;
  Coordinate& operator=(Coordinate&&) = default;
};

// convariance of enu to convariance xyz
NAVP_EXPORT NavMatrix33f64 conv_xyz(const Coordinate<CoordSystemEnum::BLH>& blh, const NavMatrix33f64&);
// convariance of enu to convariance xyz
NAVP_EXPORT NavMatrix33f64 conv_xyz(const Coordinate<CoordSystemEnum::XYZ>& xyz, const NavMatrix33f64&);

typedef Coordinate<CoordSystemEnum::XYZ> CoordinateXyz;
typedef Coordinate<CoordSystemEnum::BLH> CoordinateBlh;
typedef Coordinate<CoordSystemEnum::ENU> CoordinateEnu;

}  // namespace navp::utils