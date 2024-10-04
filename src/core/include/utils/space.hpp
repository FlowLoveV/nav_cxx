#pragma once

#include <span>
#include <vector>

#include "utils/eigen.hpp"
#include "utils/types.hpp"

namespace navp::utils {
using coordinate_t = NavVector3f64;

enum CoordSystemEnum : u8 {
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

  template <CoordSystemEnum T>
  coordinate_t operator-(const Coordinate<T>& rhs) const noexcept {
    return this->coord() - rhs.coord();
  }
};
}  // namespace details

template <>
class Coordinate<XYZ> : public details::CoordinatePayload {
 public:
  using details::CoordinatePayload::CoordinatePayload;
  Coordinate(const Coordinate&) = default;
  Coordinate(Coordinate&&) = default;
  Coordinate& operator=(const Coordinate&) = default;
  Coordinate& operator=(Coordinate&&) = default;
  // bool operator==(const Coordinate& rhs) const noexcept { return this->coord() == rhs.coord(); }
  // bool operator!=(const Coordinate&) const noexcept = default;

  auto to_blh() const noexcept -> Coordinate<BLH>;
  auto to_enu_matrix() const noexcept -> NavMatrix33f64;
  auto to_enu(const Coordinate&) const noexcept -> Coordinate<ENU>;
  auto to_enu(const std::span<Coordinate>&) const noexcept -> std::vector<Coordinate<ENU>>;
  auto to_enu(const Coordinate<BLH>&) const noexcept -> Coordinate<ENU>;
  auto to_enu(const std::span<Coordinate<BLH>>&) const noexcept -> std::vector<Coordinate<ENU>>;

  // convariance
  auto conv_enu(const NavMatrix33f64&) const -> NavMatrix33f64;
};

template <>
class Coordinate<BLH> : public details::CoordinatePayload {
 public:
  using details::CoordinatePayload::CoordinatePayload;
  Coordinate(const Coordinate&) = default;
  Coordinate(Coordinate&&) = default;
  Coordinate& operator=(const Coordinate&) = default;
  Coordinate& operator=(Coordinate&&) = default;
  // bool operator==(const Coordinate& rhs) const noexcept { return this->coord() == rhs.coord(); }
  // bool operator!=(const Coordinate&) const noexcept = default;

  auto to_xyz() const noexcept -> Coordinate<XYZ>;
  auto to_enu_matrix() const noexcept -> NavMatrix33f64;
  auto to_enu(const Coordinate&) const noexcept -> Coordinate<ENU>;
  auto to_enu(const std::span<Coordinate>&) const noexcept -> std::vector<Coordinate<ENU>>;
  auto to_enu(const Coordinate<XYZ>&) const noexcept -> Coordinate<ENU>;
  auto to_enu(const std::span<Coordinate<XYZ>>&) const noexcept -> std::vector<Coordinate<ENU>>;

  // convariance
  auto conv_enu(const NavMatrix33f64&) const noexcept -> NavMatrix33f64;
};

template <>
class Coordinate<ENU> : public details::CoordinatePayload {
 public:
  using details::CoordinatePayload::CoordinatePayload;
  Coordinate(const Coordinate&) = default;
  Coordinate(Coordinate&&) = default;
  Coordinate& operator=(const Coordinate&) = default;
  Coordinate& operator=(Coordinate&&) = default;
  // bool operator==(const Coordinate& rhs) const noexcept { return this->coord() == rhs.coord(); }
  // bool operator!=(const Coordinate&) const noexcept = default;
};

// convariance of enu to convariance xyz
extern NavMatrix33f64 conv_xyz(const Coordinate<BLH>&, const NavMatrix33f64&);
// convariance of enu to convariance xyz
extern NavMatrix33f64 conv_xyz(const Coordinate<XYZ>&, const NavMatrix33f64&);

}  // namespace navp::utils