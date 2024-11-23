#pragma once

#include "sensors/gnss/navigation.hpp"
#include "utils/eigen.hpp"
#include "utils/macro.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

// forward declaration
struct GObs;

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
  utils::NavVector3f64 pos, vel;  // default xyz
  f64 dt_trans, dtsv, fd_dtsv;    // signal transmission time 、 clock bias 、 clock speed
  f64 elevation, azimuth;

  void rotate_correct() noexcept;

  std::string format_as_string() const noexcept;
};

// features:
// 1. solve broadcast ephemeris and quary
// 2. quary tgd parameters
// 3. can be inheritted to reuse
class BrdcEphSolver {
 public:
  BrdcEphSolver() noexcept;
  explicit BrdcEphSolver(const std::vector<const Navigation*>& nav) noexcept;
  explicit BrdcEphSolver(const std::vector<const GnssNavRecord*>& gnss_record_nav) noexcept;

  // add new navigation
  void add_ephemeris(const Navigation* nav) noexcept;

  // calculate sv status at signal receive time, with signal transmission time corrected
  auto solve_sv_status(EpochUtc tr, const std::map<Sv, std::shared_ptr<GObs>>& visible_sv) noexcept -> std::vector<Sv>;
  // calculate sv status at given time, without any corrected
  auto solve_sv_status(EpochUtc tr, const std::vector<Sv>& svs) noexcept -> std::vector<Sv>;

  auto quary_sv_status(EpochUtc tr) const noexcept -> const std::map<Sv, BrdcEphResult>*;
  auto quary_sv_status(EpochUtc tr, Sv sv) const noexcept -> const BrdcEphResult*;
  auto quary_sv_status(EpochUtc tr, const std::vector<Sv>& sv) const noexcept -> std::vector<const BrdcEphResult*>;
  auto quary_sv_status_unchecked(EpochUtc tr, Sv sv) const -> const BrdcEphResult*;
  auto quary_sv_status_unchecked(EpochUtc tr, const std::vector<Sv>& sv) const -> std::vector<const BrdcEphResult*>;

  auto quary_gps_tgd(EpochUtc tr) noexcept -> const GpsGroupDelay*;
  auto quary_bds_tgd(EpochUtc tr) noexcept -> const BdsGroupDelay*;
  auto quary_gal_tgd(EpochUtc tr) noexcept -> const GalGroupDelay*;
  auto quary_glo_tgd(EpochUtc tr) noexcept -> const GloGroupDelay*;
  auto quary_qzs_tgd(EpochUtc tr) noexcept -> const QzsGroupDelay*;

 protected:
  // bool update_sv_status_pos_vel(EpochUtc t, Sv sv,
  //                               Option<std::tuple<utils::NavVector3f64, utils::NavVector3f64>>&& pv) noexcept;
  // bool update_sv_status_clock(EpochUtc t, Sv sv, const Option<std::tuple<f64, f64>>& clock) noexcept;

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

  // lanuch solver functions for different systems and ephemeris
  bool launch_ceph_solver(const utils::GTime& tr, Sv sv, f64 pr, bool correct_transmission = false) noexcept;
  bool launch_eph_solver(const utils::GTime& tr, Sv sv, f64 pr, bool correct_transmission = false) noexcept;
  bool launch_geph_solver(const utils::GTime& tr, Sv sv, f64 pr, bool correct_transmission = false) noexcept;
  bool launch_seph_solver(const utils::GTime& tr, Sv sv, f64 pr, bool correct_transmission = false) noexcept;

  std::vector<const Navigation*> nav;

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