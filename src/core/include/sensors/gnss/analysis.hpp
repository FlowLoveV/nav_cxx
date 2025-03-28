#pragma once

#include <variant>

#include "sensors/gnss/sv.hpp"
#include "utils/macro.hpp"

namespace navp::sensors::gnss {

// forward declaration
struct Sig;
struct GObs;
struct GnssObsRecord;

struct NAVP_EXPORT CombObsMeta;
struct NAVP_EXPORT RawObsMeta;        ///> raw observation meta
struct NAVP_EXPORT SDobsMeta;         ///> single-difference observation meta
struct NAVP_EXPORT DDobsMeta;         ///> double-difference observation meta
struct NAVP_EXPORT TDobsMeta;         ///> triple-difference observation meta
struct NAVP_EXPORT NdCombineObsMeta;  ///> Non differential combination observation meta
struct NAVP_EXPORT IFobsMeta;         ///> iono-free observation meta
struct NAVP_EXPORT GFobsMeta;         ///> gemetry-free observation meta
struct NAVP_EXPORT NLobsMeta;         ///> narrow-lane observation meta
struct NAVP_EXPORT WLobsMeta;         ///> wide-lane observation meta
struct NAVP_EXPORT MWobsMeta;         ///> MW observation meta
struct NAVP_EXPORT GRobsMeta;         ///> GRAPHIC observation meta

struct RawObsMeta {
  ObsCodeEnum code = ObsCodeEnum::NONE;

  auto pseudorange(const GObs& obs) const noexcept -> f64;
  auto carrier(const GObs& obs) const noexcept -> f64;
  auto snr(const GObs& obs) const noexcept -> f64;
  auto doppler(const GObs& obs) const noexcept -> f64;
};

struct SDobsMeta {
  ObsCodeEnum code;

  auto pseudorange(const GObs& obs_ref, const GObs& obs) const noexcept -> f64;
  auto carrier(const GObs& obs_ref, const GObs& obs) const noexcept -> f64;
};

struct DDobsMeta {
  ObsCodeEnum code;

  auto pseudorange(const GObs& base_obs_ref, const GObs& base_obs, const GObs& move_obs_ref,
                   const GObs& move_obs) const noexcept -> f64;
  auto carrier(const GObs& base_obs_ref, const GObs& base_obs, const GObs& move_obs_ref,
               const GObs& move_obs) const noexcept -> f64;
};

// todo
struct TDobsMeta {
  ObsCodeEnum code;

  auto pseudorange(const GObs& obs_ref, const GObs& obs) const noexcept -> f64;
  auto carrier(const GObs& obs_ref, const GObs& obs) const noexcept -> f64;
};

struct NdCombineObsMeta {
  ObsCodeEnum code1, code2;
  const GObs* obs;
  const Sig *sig1, *sig2;

  void get_sigs(const GObs& obs) noexcept;
  auto combine_pseudorange(f64 n, f64 m) const noexcept -> f64;
  auto combine_carrier(f64 n, f64 m) const noexcept -> f64;
};

struct GFobsMeta {
  ObsCodeEnum code1;
  ObsCodeEnum code2;

  auto pseudorange(const GObs& obs) const noexcept -> f64;
  auto carrier(const GObs& obs) const noexcept -> f64;
  auto pseudorange_carrier(const GObs& obs) const noexcept -> std::tuple<f64, f64>;
};

struct IFobsMeta {
  ObsCodeEnum code1;
  ObsCodeEnum code2;

  auto pseudorange(const GObs& obs) const noexcept -> f64;
  auto carrier(const GObs& obs) const noexcept -> f64;
  auto pseudorange_carrier(const GObs& obs) const noexcept -> std::tuple<f64, f64>;
};

struct NLobsMeta {
  ObsCodeEnum code1;
  ObsCodeEnum code2;

  auto carrier(const GObs& obs) const noexcept -> f64;
};

struct WLobsMeta {
  ObsCodeEnum code1;
  ObsCodeEnum code2;

  auto carrier(const GObs& obs) const noexcept -> f64;
};

struct MWobsMeta {
  ObsCodeEnum code1;
  ObsCodeEnum code2;

  auto wl_ambiguity(const GObs& obs) const noexcept -> f64;
};

struct GRobsMeta {
  ObsCodeEnum code;

  auto combine(const GObs& obs) const noexcept -> f64;
};

class DiffObsMeta : public std::variant<SDobsMeta, DDobsMeta, TDobsMeta> {
 private:
  typedef std::variant<SDobsMeta, DDobsMeta, TDobsMeta> base_type;

 public:
  using base_type::variant;
  using base_type::operator=;
  using base_type::emplace;

  auto is_sd_obs() const noexcept -> bool;
  auto is_dd_obs() const noexcept -> bool;
  auto is_td_obs() const noexcept -> bool;

 private:
  auto as_base_ptr() const noexcept -> const base_type*;
};

class CombObsMeta : public std::variant<GFobsMeta, IFobsMeta, NLobsMeta, WLobsMeta, MWobsMeta, GRobsMeta> {
 private:
  typedef std::variant<GFobsMeta, IFobsMeta, NLobsMeta, WLobsMeta, MWobsMeta, GRobsMeta> base_type;

 public:
  using base_type::variant;
  using base_type::operator=;
  using base_type::emplace;

  auto is_gf_obs() const noexcept -> bool;
  auto is_if_obs() const noexcept -> bool;
  auto is_nl_obs() const noexcept -> bool;
  auto is_wl_obs() const noexcept -> bool;
  auto is_mw_obs() const noexcept -> bool;
  auto is_gr_obs() const noexcept -> bool;

 private:
  auto as_base_ptr() const noexcept -> const base_type*;
};

}  // namespace navp::sensors::gnss