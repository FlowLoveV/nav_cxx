#include "sensors/gnss/analysis.hpp"

#include "sensors/gnss/constants.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::sensors::gnss {

// u8 glonass_freq_bias(const Navigation* nav, Sv sv, EpochUtc t) noexcept {
//   if (sv.system() != ConstellationEnum::GLO) {
//     return 0;
//   }
//   const auto& geph_map = nav->gephMap;
//   if (geph_map.contains(sv)) {
//     if (geph_map.at(sv).contains(NavMsgTypeEnum::FDMA)) {
//       for (const auto [toe, eph] : geph_map.at(sv).at(NavMsgTypeEnum::FDMA)) {
//         if (abs((static_cast<utils::GTime>(t) - toe).to_double()) <= Constants::max_toe(sv)) {
//           return eph.frq;
//         }
//       }
//     }
//   } else if (nav->glo_fcn[sv.prn - 1] > 0) {
//     return nav->glo_fcn[sv.prn - 1] - 8;
//   }
//   return 0;
// }

// Option<f64> freq(Sv sv, EpochUtc t, ObsCodeEnum code, const Navigation* nav) noexcept {
//   static constexpr f64 DFRQ1_GLO = 0.56250E6;
//   static constexpr f64 DFRQ2_GLO = 0.43750E6;
//   FreTypeEnum freq_id = Constants::code_to_freq_enum(sv.system(), code);
//   if (freq_id != FreTypeEnum::FTYPE_NONE) {
//     auto freq = Constants::frequency(freq_id);
//     // handle glonass freq bias
//     if (sv.system() == ConstellationEnum::GLO) {
//       if (freq_id == FreTypeEnum::G1 || freq_id == FreTypeEnum::G2) {
//         auto fcn = glonass_freq_bias(nav, sv, t);
//         if (fcn == 0) {
//           return None;
//         } else {
//           return freq_id == FreTypeEnum::G1 ? freq + DFRQ1_GLO * fcn : freq + DFRQ2_GLO * fcn;
//         }
//       }
//     } else {
//       return freq;
//     }
//   }
//   return None;
// }

auto RawObsMeta::pseudorange(const GObs& obs) const noexcept -> f64 {
  if (auto sig = obs.find_code(code); sig) {
    if (sig->pseudorange == 0.0) {
      nav_debug("{} {} missing {} pseudorange", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
    }
    return sig->pseudorange;
  }
  nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
  return 0.0;
}

auto RawObsMeta::carrier(const GObs& obs) const noexcept -> f64 {
  if (auto sig = obs.find_code(code); sig) {
    if (sig->carrier == 0.0) {
      nav_debug("{} {} missing {} carrier", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
    }
    return sig->carrier;
  }
  nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
  return 0.0;
}

auto RawObsMeta::snr(const GObs& obs) const noexcept -> f64 {
  if (auto sig = obs.find_code(code); sig) {
    if (sig->snr == 0.0) {
      nav_debug("{} {} missing {} snr", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
    }
    return sig->snr;
  }
  nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
  return 0.0;
}

auto RawObsMeta::doppler(const GObs& obs) const noexcept -> f64 {
  if (auto sig = obs.find_code(code); sig) {
    if (sig->doppler == 0.0) {
      nav_debug("{} {} missing {} doppler", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
    }
    return sig->doppler;
  }
  nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
  return 0.0;
}

auto SDobsMeta::pseudorange(const GObs& obs_ref, const GObs& obs) const noexcept -> f64 {
  auto meta = RawObsMeta{.code = code};
  auto ref_pse = meta.pseudorange(obs_ref);
  auto pse = meta.pseudorange(obs);
  return ref_pse != 0.0 && pse != 0.0 ? pse - ref_pse : 0.0;
}

auto SDobsMeta::carrier(const GObs& obs_ref, const GObs& obs) const noexcept -> f64 {
  auto meta = RawObsMeta{.code = code};
  auto ref_car = meta.carrier(obs_ref);
  auto car = meta.carrier(obs);
  return ref_car != 0.0 && car != 0.0 ? car - ref_car : 0.0;
}

auto DDobsMeta::pseudorange(const GObs& base_obs_ref, const GObs& base_obs, const GObs& move_obs_ref,
                            const GObs& move_obs) const noexcept -> f64 {
  auto base_sd_meta = SDobsMeta{.code = code};
  auto move_sd_meta = SDobsMeta{.code = code};
  auto base_sd_pse = base_sd_meta.pseudorange(base_obs_ref, base_obs);
  auto move_sd_pse = move_sd_meta.pseudorange(move_obs_ref, move_obs);
  return base_sd_pse != 0.0 && move_sd_pse != 0.0 ? move_sd_pse - base_sd_pse : 0.0;
}

auto DDobsMeta::carrier(const GObs& base_obs_ref, const GObs& base_obs, const GObs& move_obs_ref,
                        const GObs& move_obs) const noexcept -> f64 {
  auto base_sd_meta = SDobsMeta{.code = code};
  auto move_sd_meta = SDobsMeta{.code = code};
  auto base_sd_car = base_sd_meta.carrier(base_obs_ref, base_obs);
  auto move_sd_car = move_sd_meta.carrier(move_obs_ref, move_obs);
  return base_sd_car != 0.0 && move_sd_car != 0.0 ? move_sd_car - base_sd_car : 0.0;
}

void NdCombineObsMeta::get_sigs(const GObs& _obs) noexcept {
  obs = std::addressof(_obs);
  sig1 = _obs.find_code(code1);
  if (sig1 == nullptr) {
    nav_debug("{} {} missing {} observation", EpochUtc(_obs.time), _obs.sv, magic_enum::enum_name(code1));
  }
  sig2 = _obs.find_code(code2);
  if (sig2 == nullptr) {
    nav_debug("{} {} missing {} observation", EpochUtc(_obs.time), _obs.sv, magic_enum::enum_name(code2));
  }
}

auto NdCombineObsMeta::combine_pseudorange(f64 n, f64 m) const noexcept -> f64 {
  f64 pseudorange = n * sig1->pseudorange + m * sig2->pseudorange;
  if (sig1->pseudorange == 0.0) {
    nav_debug("{} {} missing {} pseudorange", EpochUtc(obs->time), obs->sv, magic_enum::enum_name(code1));
    pseudorange = 0.0;
  }
  if (sig2->pseudorange == 0.0) {
    nav_debug("{} {} missing {} pseudorange", EpochUtc(obs->time), obs->sv, magic_enum::enum_name(code2));
    pseudorange = 0.0;
  }
  return pseudorange;
}

auto NdCombineObsMeta::combine_carrier(f64 n, f64 m) const noexcept -> f64 {
  f64 carrier = n * sig1->carrier + m * sig2->carrier;
  if (sig1->carrier == 0.0) {
    nav_debug("{} {} missing {} carrier", EpochUtc(obs->time), obs->sv, magic_enum::enum_name(code1));
    carrier = 0.0;
  }
  if (sig2->carrier == 0.0) {
    nav_debug("{} {} missing {} carrier", EpochUtc(obs->time), obs->sv, magic_enum::enum_name(code2));
    carrier = 0.0;
  }
  return carrier;
}

auto GFobsMeta::pseudorange(const GObs& obs) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs);
  return meta.combine_pseudorange(1, -1);
}

auto GFobsMeta::carrier(const GObs& obs) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs);
  auto f1 = Constants::code_to_freq(obs.sv.system(), code1);
  auto f2 = Constants::code_to_freq(obs.sv.system(), code2);
  return meta.combine_carrier(Constants::CLIGHT / f1, -Constants::CLIGHT / f2);
}

auto GFobsMeta::pseudorange_carrier(const GObs& obs) const noexcept -> std::tuple<f64, f64> {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs);
  auto f1 = Constants::code_to_freq(obs.sv.system(), code1);
  auto f2 = Constants::code_to_freq(obs.sv.system(), code2);
  return {meta.combine_pseudorange(1, -1), meta.combine_carrier(Constants::CLIGHT / f1, -Constants::CLIGHT / f2)};
}

auto IFobsMeta::pseudorange(const GObs& obs) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs);
  auto f1 = Constants::code_to_freq(obs.sv.system(), code1);
  auto f2 = Constants::code_to_freq(obs.sv.system(), code2);
  auto f1_2 = f1 * f1, f2_2 = f2 * f2;
  auto f = f1_2 - f2_2;
  return meta.combine_pseudorange(f1_2 / f, -f2_2 / f);
}

auto IFobsMeta::carrier(const GObs& obs) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs);
  auto f1 = Constants::code_to_freq(obs.sv.system(), code1);
  auto f2 = Constants::code_to_freq(obs.sv.system(), code2);
  auto f1_2 = f1 * f1, f2_2 = f2 * f2;
  auto f = f1_2 - f2_2;
  return meta.combine_carrier(f1_2 / f, -f1 * f2 / f);
}

auto IFobsMeta::pseudorange_carrier(const GObs& obs) const noexcept -> std::tuple<f64, f64> {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs);
  auto f1 = Constants::code_to_freq(obs.sv.system(), code1);
  auto f2 = Constants::code_to_freq(obs.sv.system(), code2);
  auto f1_2 = f1 * f1, f2_2 = f2 * f2;
  auto f = f1_2 - f2_2;
  return {meta.combine_pseudorange(f1_2 / f, -f2_2 / f), meta.combine_carrier(f1_2 / f, -f1 * f2 / f)};
}

auto NLobsMeta::carrier(const GObs& obs) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs);
  return meta.combine_pseudorange(1, 1);
}

auto WLobsMeta::carrier(const GObs& obs) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs);
  return meta.combine_pseudorange(1, -1);
}

auto GRobsMeta::combine(const GObs& obs) const noexcept -> f64 {
  auto sig = obs.find_code(code);
  if (sig == nullptr) {
    nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
    return 0.0;
  }
  if (sig->pseudorange == 0.0) {
    nav_debug("{} {} missing {} pseudorange", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
    return 0.0;
  }
  if (sig->carrier == 0.0) {
    nav_debug("{} {} missing {} carrier", EpochUtc(obs.time), obs.sv, magic_enum::enum_name(code));
    return 0.0;
  }
  auto f = Constants::code_to_freq(obs.sv.system(), code);
  return 0.5 * (Constants::CLIGHT / f * sig->carrier + sig->pseudorange);
}

auto MWobsMeta::wl_ambiguity(const GObs& obs) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs);
  auto f1 = Constants::code_to_freq(obs.sv.system(), code1);
  auto f2 = Constants::code_to_freq(obs.sv.system(), code2);
  auto lambda_wl = Constants::CLIGHT / (f1 - f2);
  auto combine_pse = meta.combine_pseudorange(f1 / (f1 + f2) / lambda_wl, f2 / (f1 + f2) / lambda_wl);
  auto combine_car = meta.combine_carrier(1, -1);
  if (combine_car == 0.0 || combine_pse == 0.0) {
    return 0.0;
  }
  return combine_pse - combine_car;
}

auto CombObsMeta::as_base_ptr() const noexcept -> const base_type* { return static_cast<const base_type*>(this); }

auto DiffObsMeta::as_base_ptr() const noexcept -> const base_type* { return static_cast<const base_type*>(this); }

auto DiffObsMeta::is_sd_obs() const noexcept -> bool { return std::holds_alternative<SDobsMeta>(*as_base_ptr()); }

auto DiffObsMeta::is_dd_obs() const noexcept -> bool { return std::holds_alternative<DDobsMeta>(*as_base_ptr()); }

auto DiffObsMeta::is_td_obs() const noexcept -> bool { return std::holds_alternative<TDobsMeta>(*as_base_ptr()); }

auto CombObsMeta::is_gf_obs() const noexcept -> bool { return std::holds_alternative<GFobsMeta>(*as_base_ptr()); }

auto CombObsMeta::is_if_obs() const noexcept -> bool { return std::holds_alternative<IFobsMeta>(*as_base_ptr()); }

auto CombObsMeta::is_nl_obs() const noexcept -> bool { return std::holds_alternative<NLobsMeta>(*as_base_ptr()); }

auto CombObsMeta::is_wl_obs() const noexcept -> bool { return std::holds_alternative<WLobsMeta>(*as_base_ptr()); }

auto CombObsMeta::is_mw_obs() const noexcept -> bool { return std::holds_alternative<MWobsMeta>(*as_base_ptr()); }

auto CombObsMeta::is_gr_obs() const noexcept -> bool { return std::holds_alternative<GRobsMeta>(*as_base_ptr()); }

// auto CombObsMeta::is_difference_obs() const noexcept -> bool { return is_sd_obs() || is_dd_obs() || is_td_obs(); }

// auto CombObsMeta::is_combination_obs() const noexcept -> bool {
//   return is_gf_obs() || is_if_obs() || is_nl_obs() || is_wl_obs() || is_mw_obs() || is_gr_obs();
// }

}  // namespace navp::sensors::gnss