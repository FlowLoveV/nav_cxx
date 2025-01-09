#pragma once

#include "sensors/gnss/navigation.hpp"
#include "utils/eigen.hpp"
#include "utils/macro.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

// forward declaration
struct Navigation;

class PreciseEphSolver;
struct PreciseEphResult;
struct PreciseEphSettings;

struct NAVP_EXPORT PreciseEphResult {
  Sv sv;
  utils::NavVector3f64 pos, vel, pos_std, vel_std;

  // dcb pcv pco ...etc
};

struct PreciseEphSettings {
  u8 nmax = 10;
  f64 exterr_clk = 1e-3, exterr_eph = 5e-7, max_dte = 900.0;
};

class NAVP_EXPORT PreciseEphSolver {
 public:
  explicit PreciseEphSolver(const std::vector<const Navigation*>& nav) noexcept;
  explicit PreciseEphSolver(const std::vector<const GnssNavRecord*>& gnss_record_nav) noexcept;

  auto solve_sv_status(EpochUtc t, const std::vector<Sv>& svs) noexcept -> std::vector<Sv>;

  auto quary_sv_status(EpochUtc t, Sv sv) const noexcept -> const PreciseEphResult*;
  auto quary_sv_status(EpochUtc t, const std::vector<Sv>& sv) const noexcept -> std::vector<const PreciseEphResult*>;
  auto quary_sv_status_unchecked(EpochUtc t, const std::vector<Sv>& sv) const -> std::vector<const PreciseEphResult*>;
  auto quary_sv_status_unchecked(EpochUtc t, Sv sv) const -> const PreciseEphResult*;

  // settings

 protected:
  auto solve_sv_pephpos(EpochUtc t, Sv sv) noexcept -> bool;
  auto solve_sv_pephpos(EpochUtc t, const std::vector<Sv>& svs) noexcept -> std::vector<Sv>;

  std::vector<const Navigation*> nav;
  PreciseEphSettings settings;
  std::map<EpochUtc, std::map<Sv, PreciseEphResult>> sv_status;
};

}  // namespace navp::sensors::gnss