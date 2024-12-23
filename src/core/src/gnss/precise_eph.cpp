#include "sensors/gnss/precise_eph.hpp"

#include <ranges>

namespace navp::sensors::gnss {

/** polynomial interpolation by Neville's algorithm
 */
f64 interpolate(const f64 *x, f64 *y, int n) {
  for (int j = 1; j < n; j++) {
    for (int i = 0; i < n - j; ++i) {
      y[i] = (x[i + j] * y[i] - x[i] * y[i + 1]) / (x[i + j] - x[i]);
    }
  }
  return y[0];
}

PreciseEphSolver::PreciseEphSolver(const std::vector<const Navigation *> &_nav) noexcept : nav(std::move(_nav)) {}

PreciseEphSolver::PreciseEphSolver(const std::vector<const GnssNavRecord *> &record_gnss_nav) noexcept
    : PreciseEphSolver(record_gnss_nav |
                       std::views::transform([](const GnssNavRecord *record) { return record->nav.get(); }) |
                       std::ranges::to<std::vector<const Navigation *>>()) {}

auto PreciseEphSolver::solve_sv_status(EpochUtc t, const std::vector<Sv> &svs) noexcept -> std::vector<Sv> {
  std::vector<Sv> solved_sv;

  // todo

  return solved_sv;
}

auto PreciseEphSolver::solve_sv_pephpos(EpochUtc t, Sv sv) noexcept -> bool {
  // find target precise ephemeris map
  auto nav_it = std::ranges::find_if(nav, [&](const Navigation *_nav) { return _nav->pephMap.contains(sv); });
  if (nav_it == nav.end()) {
    nav_warn("No precise position found at {} for {}", t, sv);
    return false;
  }
  auto &peph_map = (*nav_it)->pephMap.at(sv);

  auto bt = static_cast<EpochUtc>(peph_map.begin()->first);
  auto et = static_cast<EpochUtc>(peph_map.rbegin()->first);

  // if (peph_map.size() < settings.nmax + 1 || t < bt - settings.max_dte || t > et + settings.max_dte) {
  //   nav_warn("No precise ephemeris for {} at {},ephemerides cover {} to {}", sv, t, bt, et);
  //   return false;
  // }

  // find closest peph
  auto peph_it = peph_map.lower_bound(static_cast<utils::GTime>(t));
  if (peph_it == peph_map.end()) {
    peph_it--;
  }
  auto middle0 = peph_it;
  // find the appropriate interpolation interval
  for (int i = 0; i < settings.nmax / 2; ++i) {
    peph_it++;
    if (peph_it == peph_map.end()) {
      break;
    }
  }
  for (int i = 0; i <= settings.nmax; ++i) {
    peph_it--;
    if (peph_it == peph_map.begin()) {
      break;
    }
  }
  auto begin_it = peph_it;

  std::vector<f64> t_vec(settings.nmax + 1);
  std::vector<utils::NavVector3f64> p_vec(settings.nmax + 1);
  f64 c[2], s[3];

  for (int i = 0; i < settings.nmax; ++i, begin_it++) {
    auto &peph = begin_it->second;
    if (peph.pos.isZero()) {
      return false;
    }
  }
  return true;
}

auto PreciseEphSolver::solve_sv_pephpos(EpochUtc t, const std::vector<Sv> &svs) noexcept -> std::vector<Sv> {
  std::vector<Sv> solved_sv;

  // todo

  return solved_sv;
}
}  // namespace navp::sensors::gnss