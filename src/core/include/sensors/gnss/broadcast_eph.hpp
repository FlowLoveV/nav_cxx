#pragma once

#include "sensors/gnss/navigation.hpp"
#include "utils/eigen.hpp"
#include "utils/macro.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

class NAVP_EXPORT BrdcEphSolver;

// Group Delay and Inter-Satellite Clock
struct NAVP_EXPORT BdsGroupDelay;
struct NAVP_EXPORT GpsGroupDelay;
struct NAVP_EXPORT GalGroupDelay;
struct NAVP_EXPORT QzsGroupDelay;
struct NAVP_EXPORT GloGroupDelay;
// A structure representing satellite status information
struct NAVP_EXPORT BrdcEphResult {
  Sv sv;
  utils::NavVector3f64 pos;  // default xyz
  utils::NavVector3f64 vel;  // default xyz
  f64 dtsv, fd_dtsv;
  f64 elevation, azimuth;

  std::string format_as_string() const noexcept;
};

class BrdcEphSolver {
 public:
  BrdcEphSolver(const Navigation* nav) noexcept;
  BrdcEphSolver(const GnssNavRecord& record_gnss_nav) noexcept;

  // calculate sv status at given epoch
  auto solve_sv_stat(EpochUtc t, const std::vector<Sv>& sv) noexcept -> std::vector<Sv>;

  auto quary_sv_status(EpochUtc t, Sv sv) const noexcept -> const BrdcEphResult*;
  auto quary_sv_status(EpochUtc t, const std::vector<Sv>& sv) const noexcept -> std::vector<const BrdcEphResult*>;
  auto quary_sv_status_unchecked(EpochUtc t, const std::vector<Sv>& sv) const -> std::vector<const BrdcEphResult*>;
  auto quary_sv_status_unchecked(EpochUtc t, Sv sv) const -> const BrdcEphResult*;

  auto quary_gps_tgd(EpochUtc t) noexcept -> const GpsGroupDelay*;
  auto quary_bds_tgd(EpochUtc t) noexcept -> const BdsGroupDelay*;
  auto quary_gal_tgd(EpochUtc t) noexcept -> const GalGroupDelay*;
  auto quary_glo_tgd(EpochUtc t) noexcept -> const GloGroupDelay*;
  auto quary_qzs_tgd(EpochUtc t) noexcept -> const QzsGroupDelay*;

  // quary freqency bias(specliall for glonass)
  // for other system,return 0
  auto quary_frq_bias(Sv sv, EpochUtc t) const noexcept -> i32;

  // quary frequency

 protected:
  bool update_sv_status_pos_vel(EpochUtc t, Sv sv,
                                Option<std::tuple<utils::NavVector3f64, utils::NavVector3f64>>&& pv) noexcept;
  bool update_sv_status_clock(EpochUtc t, Sv sv, const Option<std::tuple<f64, f64>>& clock) noexcept;
  bool update_newest_gps_lnav(const utils::GTime& t) noexcept;
  bool update_newest_gps_cnav(const utils::GTime& t) noexcept;
  bool update_newest_gps_cnv2(const utils::GTime& t) noexcept;
  bool update_newest_bds_d1d2(const utils::GTime& t) noexcept;
  bool update_newest_bds_cnv1(const utils::GTime& t) noexcept;
  bool update_newest_bds_cnv2(const utils::GTime& t) noexcept;
  bool update_newest_bds_cnv3(const utils::GTime& t) noexcept;
  bool update_newest_gal_ifnav(const utils::GTime& t) noexcept;
  bool update_newest_qzs_lnav(const utils::GTime& t) noexcept;
  bool update_newest_qzs_cnav(const utils::GTime& t) noexcept;
  bool update_newest_qzs_cnv2(const utils::GTime& t) noexcept;
  bool update_newest_glo_fdma(const utils::GTime& t) noexcept;
  // Update tgd parameters
  void update_tgd(ConstellationEnum sys, const utils::GTime& t) noexcept;

  const Navigation* nav;

  // sv status
  std::shared_ptr<std::map<EpochUtc, std::map<Sv, BrdcEphResult>>> sv_status = nullptr;

  // cache newest ephemeris of different versions and system
  Eph *cache_bds_d1d2, *cache_gps_lnav, *cache_qzs_lnav, *cache_gal_ifnav;
  Ceph *cache_bds_cnv1, *cache_bds_cnv2, *cache_bds_cnv3, *cache_gps_cnav, *cache_gps_cnv2, *cache_qzs_cnav,
      *cache_qzs_cnv2;
  Geph* cache_glo_fdma;

  // tgd paramenters
  std::shared_ptr<BdsGroupDelay> bds_gd = nullptr;
  std::shared_ptr<GpsGroupDelay> gps_gd = nullptr;
  std::shared_ptr<GalGroupDelay> gal_gd = nullptr;
  std::shared_ptr<QzsGroupDelay> qzs_gd = nullptr;
  std::shared_ptr<GloGroupDelay> glo_gd = nullptr;
};

}  // namespace navp::sensors::gnss