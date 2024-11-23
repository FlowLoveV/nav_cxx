#include "sensors/gnss/analysis.hpp"

#include "ginan/constants.hpp"
#include "sensors/gnss/constants.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"

namespace navp::sensors::gnss {

u8 glonass_freq_bias(const Navigation* nav, Sv sv, EpochUtc t) noexcept {
  if (sv.constellation.id != ConstellationEnum::GLO) {
    return 0;
  }
  const auto& geph_map = nav->gephMap;
  if (geph_map.contains(sv)) {
    if (geph_map.at(sv).contains(NavMsgTypeEnum::FDMA)) {
      for (const auto [toe, eph] : geph_map.at(sv).at(NavMsgTypeEnum::FDMA)) {
        if (abs((static_cast<utils::GTime>(t) - toe).to_double()) <= Constants::max_toe(sv)) {
          return eph.frq;
        }
      }
    }
  } else if (nav->glo_fcn[sv.prn - 1] > 0) {
    return nav->glo_fcn[sv.prn - 1] - 8;
  }
  return 0;
}

Option<f64> freq(Sv sv, EpochUtc t, ObsCodeEnum code, const Navigation* nav) noexcept {
  using ginan::code2Freq;
  using ginan::roughFrequency;
  static constexpr f64 DFRQ1_GLO = 0.56250E6;
  static constexpr f64 DFRQ2_GLO = 0.43750E6;
  auto constellation = sv.constellation.id;
  if (code2Freq.contains(constellation) && code2Freq[constellation].contains(code)) {
    auto freq_id = code2Freq[constellation][code];
    auto frequency = roughFrequency[freq_id];
    // handle glonass freq bias
    if (constellation == ConstellationEnum::GLO) {
      if (freq_id == FreTypeEnum::G1 || freq_id == FreTypeEnum::G2) {
        auto fcn = glonass_freq_bias(nav, sv, t);
        if (fcn == 0) {
          return None;
        } else {
          return freq_id == FreTypeEnum::G1 ? frequency + DFRQ1_GLO * fcn : frequency + DFRQ2_GLO * fcn;
        }
      }
    } else {
      return frequency;
    }
  }
  return None;
}

auto RawObsMeta::pseudorange(const GObs& obs) const noexcept -> f64 {
  if (auto sig = obs.find_code(code); sig) {
    if (sig->P == 0.0) {
      nav_debug("{} {} missing {} pseudorange", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
    }
    return sig->P;
  }
  nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
  return 0.0;
}

auto RawObsMeta::carrier(const GObs& obs) const noexcept -> f64 {
  if (auto sig = obs.find_code(code); sig) {
    if (sig->L == 0.0) {
      nav_debug("{} {} missing {} carrier", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
    }
    return sig->L;
  }
  nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
  return 0.0;
}

auto RawObsMeta::snr(const GObs& obs) const noexcept -> f64 {
  if (auto sig = obs.find_code(code); sig) {
    if (sig->snr == 0.0) {
      nav_debug("{} {} missing {} snr", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
    }
    return sig->snr;
  }
  nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
  return 0.0;
}

auto RawObsMeta::doppler(const GObs& obs) const noexcept -> f64 {
  if (auto sig = obs.find_code(code); sig) {
    if (sig->D == 0.0) {
      nav_debug("{} {} missing {} doppler", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
    }
    return sig->D;
  }
  nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
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
  auto base_sd_meta = SDobsMeta{.code = code1};
  auto move_sd_meta = SDobsMeta{.code = code2};
  auto base_sd_pse = base_sd_meta.pseudorange(base_obs_ref, base_obs);
  auto move_sd_pse = move_sd_meta.pseudorange(move_obs_ref, move_obs);
  return base_sd_pse != 0.0 && move_sd_pse != 0.0 ? move_sd_pse - base_sd_pse : 0.0;
}

auto DDobsMeta::carrier(const GObs& base_obs_ref, const GObs& base_obs, const GObs& move_obs_ref,
                        const GObs& move_obs) const noexcept -> f64 {
  auto base_sd_meta = SDobsMeta{.code = code1};
  auto move_sd_meta = SDobsMeta{.code = code2};
  auto base_sd_car = base_sd_meta.carrier(base_obs_ref, base_obs);
  auto move_sd_car = move_sd_meta.carrier(move_obs_ref, move_obs);
  return base_sd_car != 0.0 && move_sd_car != 0.0 ? move_sd_car - base_sd_car : 0.0;
}

void NdCombineObsMeta::get_sigs(const GObs& _obs1, const GObs& _obs2) noexcept {
  obs1 = &_obs1, obs2 = &_obs2;
  sig1 = obs1->find_code(code1);
  if (sig1 == nullptr) {
    nav_debug("{} {} missing {} observation", EpochUtc(obs1->time), obs1->Sat, magic_enum::enum_name(code1));
  }
  sig2 = obs2->find_code(code2);
  if (sig2 == nullptr) {
    nav_debug("{} {} missing {} observation", EpochUtc(obs2->time), obs2->Sat, magic_enum::enum_name(code2));
  }
}

auto NdCombineObsMeta::combine_pseudorange(f64 n, f64 m) const noexcept -> f64 {
  f64 pseudorange = n * sig1->P + m * sig2->P;
  if (sig1->P == 0.0) {
    nav_debug("{} {} missing {} pseudorange", EpochUtc(obs1->time), obs1->Sat, magic_enum::enum_name(code1));
    pseudorange = 0.0;
  }
  if (sig2->P == 0.0) {
    nav_debug("{} {} missing {} pseudorange", EpochUtc(obs2->time), obs2->Sat, magic_enum::enum_name(code2));
    pseudorange = 0.0;
  }
  return pseudorange;
}

auto NdCombineObsMeta::combine_carrier(f64 n, f64 m) const noexcept -> f64 {
  f64 carrier = n * sig1->L + m * sig2->L;
  if (sig1->L == 0.0) {
    nav_debug("{} {} missing {} carrier", EpochUtc(obs1->time), obs1->Sat, magic_enum::enum_name(code1));
    carrier = 0.0;
  }
  if (sig2->L == 0.0) {
    nav_debug("{} {} missing {} carrier", EpochUtc(obs2->time), obs2->Sat, magic_enum::enum_name(code2));
    carrier = 0.0;
  }
  return carrier;
}

auto GFobsMeta::pseudorange(const GObs& obs1, const GObs obs2) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs1, obs2);
  return meta.combine_pseudorange(1, -1);
}

auto GFobsMeta::carrier(const GObs& obs1, const GObs obs2, const Navigation* nav) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs1, obs2);
  auto f1 = freq(meta.obs1->Sat, static_cast<EpochUtc>(meta.obs1->time), code1, nav).unwrap();
  auto f2 = freq(meta.obs2->Sat, static_cast<EpochUtc>(meta.obs2->time), code2, nav).unwrap();
  return meta.combine_carrier(Constants::CLIGHT / f1, -Constants::CLIGHT / f2);
}

auto GFobsMeta::pseudorange_carrier(const GObs& obs1, const GObs& obs2,
                                    const Navigation* nav) const noexcept -> std::tuple<f64, f64> {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs1, obs2);
  auto f1 = freq(meta.obs1->Sat, static_cast<EpochUtc>(meta.obs1->time), code1, nav).unwrap();
  auto f2 = freq(meta.obs2->Sat, static_cast<EpochUtc>(meta.obs2->time), code2, nav).unwrap();
  return {meta.combine_pseudorange(1, -1), meta.combine_carrier(Constants::CLIGHT / f1, -Constants::CLIGHT / f2)};
}

auto IFobsMeta::pseudorange(const GObs& obs1, const GObs obs2, const Navigation* nav) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs1, obs2);
  auto f1 = freq(meta.obs1->Sat, static_cast<EpochUtc>(meta.obs1->time), code1, nav).unwrap();
  auto f2 = freq(meta.obs2->Sat, static_cast<EpochUtc>(meta.obs2->time), code2, nav).unwrap();
  auto f1_2 = f1 * f1, f2_2 = f2 * f2;
  auto f = f1_2 - f2_2;
  return meta.combine_pseudorange(f1_2 / f, -f2_2 / f);
}

auto IFobsMeta::carrier(const GObs& obs1, const GObs obs2, const Navigation* nav) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs1, obs2);
  auto f1 = freq(meta.obs1->Sat, static_cast<EpochUtc>(meta.obs1->time), code1, nav).unwrap();
  auto f2 = freq(meta.obs2->Sat, static_cast<EpochUtc>(meta.obs2->time), code2, nav).unwrap();
  auto f1_2 = f1 * f1, f2_2 = f2 * f2;
  auto f = f1_2 - f2_2;
  return meta.combine_carrier(f1_2 / f, -f1 * f2 / f);
}

auto IFobsMeta::pseudorange_carrier(const GObs& obs1, const GObs& obs2,
                                    const Navigation* nav) const noexcept -> std::tuple<f64, f64> {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs1, obs2);
  auto f1 = freq(meta.obs1->Sat, static_cast<EpochUtc>(meta.obs1->time), code1, nav).unwrap();
  auto f2 = freq(meta.obs2->Sat, static_cast<EpochUtc>(meta.obs2->time), code2, nav).unwrap();
  auto f1_2 = f1 * f1, f2_2 = f2 * f2;
  auto f = f1_2 - f2_2;
  return {meta.combine_pseudorange(f1_2 / f, -f2_2 / f), meta.combine_carrier(f1_2 / f, -f1 * f2 / f)};
}

auto NLobsMeta::carrier(const GObs& obs1, const GObs obs2) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs1, obs2);
  return meta.combine_pseudorange(1, 1);
}

auto WLobsMeta::carrier(const GObs& obs1, const GObs obs2) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs1, obs2);
  return meta.combine_pseudorange(1, -1);
}

auto GRobsMeta::combine(const GObs& obs, const Navigation* nav) const noexcept -> f64 {
  auto sig = obs.find_code(code);
  if (sig == nullptr) {
    nav_debug("{} {} missing {} observation", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
    return 0.0;
  }
  if (sig->P == 0.0) {
    nav_debug("{} {} missing {} pseudorange", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
    return 0.0;
  }
  if (sig->L == 0.0) {
    nav_debug("{} {} missing {} carrier", EpochUtc(obs.time), obs.Sat, magic_enum::enum_name(code));
    return 0.0;
  }
  auto f = freq(obs.Sat, static_cast<EpochUtc>(obs.time), code, nav).unwrap();
  return 0.5 * (Constants::CLIGHT / f * sig->L + sig->P);
}

auto MWobsMeta::wl_ambiguity(const GObs& obs1, const GObs obs2, const Navigation* nav) const noexcept -> f64 {
  auto meta = NdCombineObsMeta{.code1 = code1, .code2 = code2};
  meta.get_sigs(obs1, obs2);
  auto f1 = freq(meta.obs1->Sat, static_cast<EpochUtc>(meta.obs1->time), code1, nav).unwrap();
  auto f2 = freq(meta.obs2->Sat, static_cast<EpochUtc>(meta.obs2->time), code2, nav).unwrap();
  auto lambda_wl = Constants::CLIGHT / (f1 - f2);
  auto combine_pse = meta.combine_pseudorange(f1 / (f1 + f2) / lambda_wl, f2 / (f1 + f2) / lambda_wl);
  auto combine_car = meta.combine_carrier(1, -1);
  if (combine_car == 0.0 || combine_pse == 0.0) {
    return 0.0;
  }
  return combine_pse - combine_car;
}

auto ObsMeta::as_base_ptr() const noexcept -> const base_type* { return static_cast<const base_type*>(this); }

auto ObsMeta::is_raw_obs() const noexcept -> bool { return std::holds_alternative<RawObsMeta>(*as_base_ptr()); }

auto ObsMeta::is_sd_obs() const noexcept -> bool { return std::holds_alternative<SDobsMeta>(*as_base_ptr()); }

auto ObsMeta::is_dd_obs() const noexcept -> bool { return std::holds_alternative<DDobsMeta>(*as_base_ptr()); }

auto ObsMeta::is_td_obs() const noexcept -> bool { return std::holds_alternative<TDobsMeta>(*as_base_ptr()); }

auto ObsMeta::is_gf_obs() const noexcept -> bool { return std::holds_alternative<GFobsMeta>(*as_base_ptr()); }

auto ObsMeta::is_if_obs() const noexcept -> bool { return std::holds_alternative<IFobsMeta>(*as_base_ptr()); }

auto ObsMeta::is_nl_obs() const noexcept -> bool { return std::holds_alternative<NLobsMeta>(*as_base_ptr()); }

auto ObsMeta::is_wl_obs() const noexcept -> bool { return std::holds_alternative<WLobsMeta>(*as_base_ptr()); }

auto ObsMeta::is_mw_obs() const noexcept -> bool { return std::holds_alternative<MWobsMeta>(*as_base_ptr()); }

auto ObsMeta::is_gr_obs() const noexcept -> bool { return std::holds_alternative<GRobsMeta>(*as_base_ptr()); }

auto ObsMeta::is_difference_obs() const noexcept -> bool { return is_sd_obs() || is_dd_obs() || is_td_obs(); }

auto ObsMeta::is_combination_obs() const noexcept -> bool {
  return is_gf_obs() || is_if_obs() || is_nl_obs() || is_wl_obs() || is_mw_obs() || is_gr_obs();
}

}  // namespace navp::sensors::gnss