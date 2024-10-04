#pragma once

#include <Eigen/Eigen>

#include "sensors/gnss/navigation.hpp"
#include "utils/eigen.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

class BrdcEphSolver;

namespace details {
// Group Delay and Inter-Satellite Clock
struct BdsGroupDelay;
struct GpsGroupDelay;
struct GalGroupDelay;
struct QzsGroupDelay;
struct GloGroupDelay;
// A structure representing satellite status information
struct BrdcEphResult {
  Sv sv;
  utils::NavVector3f64 pos;  // default xyz
  utils::NavVector3f64 vel;  // default xyz
  f64 dtsv, fd_dtsv;
  f64 elevation, azimuth;

  std::string format_as_string() const noexcept;
};
}  // namespace details

class BrdcEphSolver {
 public:
  BrdcEphSolver(const Navigation* nav) noexcept;
  BrdcEphSolver(const RecordGnssNav& record_gnss_nav) noexcept;

  // calculate sv status at given epoch
  auto solve_sv_stat(const std::vector<Sv>& sv, Epoch<UTC> t) noexcept -> std::vector<Sv>;

  // Update tgd parameters
  void update_tgd(ConstellationEnum sys, const utils::GTime& t) noexcept;

  // quary sv status
  auto quary_sv_status(Sv sv, Epoch<UTC> t) const noexcept -> const details::BrdcEphResult*;
  auto quary_sv_status(const std::vector<Sv>& sv,
                       Epoch<UTC> t) const noexcept -> std::vector<const details::BrdcEphResult*>;
  auto quary_sv_status_unchecked(const std::vector<Sv>& sv,
                                 Epoch<UTC> t) const -> std::vector<const details::BrdcEphResult*>;
  auto quary_sv_status_unchecked(Sv sv, Epoch<UTC> t) const -> const details::BrdcEphResult*;

  // tgd paramenters
  std::shared_ptr<details::BdsGroupDelay> bds_gd = nullptr;
  std::shared_ptr<details::GpsGroupDelay> gps_gd = nullptr;
  std::shared_ptr<details::GalGroupDelay> gal_gd = nullptr;
  std::shared_ptr<details::QzsGroupDelay> qzs_gd = nullptr;
  std::shared_ptr<details::GloGroupDelay> glo_gd = nullptr;

 protected:
  bool update_sv_status_pos_vel(Epoch<UTC> t, Sv sv,
                                Option<std::tuple<utils::NavVector3f64, utils::NavVector3f64>>&& pv) noexcept;
  bool update_sv_status_clock(Epoch<UTC> t, Sv sv, const Option<std::tuple<f64, f64>>& clock) noexcept;
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

  const Navigation* nav;

  // sv status
  std::shared_ptr<std::map<Epoch<UTC>, std::map<Sv, details::BrdcEphResult>>> sv_status = nullptr;

  // cache newest ephemeris of different versions and system
  Eph *cache_bds_d1d2, *cache_gps_lnav, *cache_qzs_lnav, *cache_gal_ifnav;
  Ceph *cache_bds_cnv1, *cache_bds_cnv2, *cache_bds_cnv3, *cache_gps_cnav, *cache_gps_cnv2, *cache_qzs_cnav,
      *cache_qzs_cnv2;
  Geph* cache_glo_fdma;
};

}  // namespace navp::sensors::gnss