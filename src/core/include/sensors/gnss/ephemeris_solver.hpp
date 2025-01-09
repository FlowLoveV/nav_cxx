#pragma once

#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "utils/macro.hpp"
#include "utils/space.hpp"
#include "utils/time.hpp"

namespace navp::sensors::gnss {

class EphemerisSolver;

// Group Delay and Inter-Satellite Clock
struct BdsGroupDelay;
struct GpsGroupDelay;
struct GalGroupDelay;
struct QzsGroupDelay;
struct GloGroupDelay;
// A structure representing satellite status information
struct NAVP_EXPORT EphemerisResult {
  Sv sv;
  utils::CoordinateXyz pos, vel;   // satellite position (ecef) {x,y,z} (m)
  f64 var;                         // satellite position and clock variance (m^2)
  f64 dt_trans, dtsv, fd_dtsv;     // signal transmission time(s) 、 clock bias(s) 、 clock speed
  mutable f64 elevation, azimuth;  // satellite elevation、azimuth （rad)

  void rotate_correct() noexcept;

  void update_ea_from(const utils::CoordinateXyz& pos) const noexcept;

  std::string format_as_string() const noexcept;

  void view_vector_to(const utils::CoordinateXyz& station_pos, f64& x, f64& y, f64& z, f64& distance) const noexcept;
};

// features:
// 1. solve broadcast ephemeris and quary
// 2. quary tgd parameters
// 3. can be inheritted to reuse
class NAVP_EXPORT EphemerisSolver {
 public:
  using SvMap = std::unordered_map<Sv, EphemerisResult>;
  using TimeSvMap = std::map<EpochUtc, SvMap>;

  EphemerisSolver(std::shared_ptr<spdlog::logger> logger) noexcept;

  // set satellites max storage
  EphemerisSolver& set_storage(i32 storage) noexcept;

  // add new navigation
  void add_ephemeris(const Navigation* nav) noexcept;

  // calculate sv status at signal receive time, with signal transmission time corrected
  auto solve_sv_status(EpochUtc tr, const GnssObsRecord::ObsMap* visible_sv) noexcept -> std::vector<Sv>;
  // calculate sv status at given time, without any corrected
  auto solve_sv_status(EpochUtc tr, const std::vector<Sv>& svs) noexcept -> std::vector<Sv>;

  auto quary_sv_status(EpochUtc tr) const noexcept -> const SvMap*;
  auto quary_sv_status(EpochUtc tr, Sv sv) const noexcept -> const EphemerisResult*;
  auto quary_sv_status(EpochUtc tr, const std::vector<Sv>& sv) const noexcept -> std::vector<const EphemerisResult*>;
  auto quary_sv_status_unchecked(EpochUtc tr, Sv sv) const -> const EphemerisResult*;
  auto quary_sv_status_unchecked(EpochUtc tr, const std::vector<Sv>& sv) const -> std::vector<const EphemerisResult*>;

  template <typename Func>
  void for_each_sv_at(EpochUtc tr, Func&& func) {
    if (sv_status_->contains(tr)) {
      for (auto&& [_, status] : sv_status_->at(tr)) {
        std::invoke(std::forward<Func>(func), std::forward<EphemerisResult>(status));
      }
    }
  }

  auto quary_gps_tgd(EpochUtc tr) noexcept -> const GpsGroupDelay*;
  auto quary_bds_tgd(EpochUtc tr) noexcept -> const BdsGroupDelay*;
  auto quary_gal_tgd(EpochUtc tr) noexcept -> const GalGroupDelay*;
  auto quary_glo_tgd(EpochUtc tr) noexcept -> const GloGroupDelay*;
  auto quary_qzs_tgd(EpochUtc tr) noexcept -> const QzsGroupDelay*;

 protected:
  bool brdc_solve_sv_status(EpochUtc tr, Sv sv, f64 pr) noexcept;
  bool precise_solve_sv_status(EpochUtc tr, Sv sv, f64 pr) noexcept;

  // tgd helper functions
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

  // lanuch brdc solver functions for different systems and ephemeris
  bool launch_ceph_solver(const utils::GTime& tr, Sv sv, f64 pr, bool correct_transmission = false) noexcept;
  bool launch_eph_solver(const utils::GTime& tr, Sv sv, f64 pr, bool correct_transmission = false) noexcept;
  bool launch_geph_solver(const utils::GTime& tr, Sv sv, f64 pr, bool correct_transmission = false) noexcept;
  bool launch_seph_solver(const utils::GTime& tr, Sv sv, f64 pr, bool correct_transmission = false) noexcept;

  std::vector<const Navigation*> nav_vec_;

  void erase() noexcept;

  std::unique_ptr<TimeSvMap> sv_status_ = nullptr;  // sv status
  i32 storage_ = -1;                                // satellite status storage

  // cache newest ephemeris of different versions and system
  Eph *cache_bds_d1d2_, *cache_gps_lnav_, *cache_qzs_lnav_, *cache_gal_ifnav_;
  Ceph *cache_bds_cnv1_, *cache_bds_cnv2_, *cache_bds_cnv3_, *cache_gps_cnav_, *cache_gps_cnv2_, *cache_qzs_cnav_,
      *cache_qzs_cnv2_;
  Geph* cache_glo_fdma_;

  // tgd paramenters
  std::shared_ptr<BdsGroupDelay> bds_gd_ = nullptr;
  std::shared_ptr<GpsGroupDelay> gps_gd_ = nullptr;
  std::shared_ptr<GalGroupDelay> gal_gd_ = nullptr;
  std::shared_ptr<QzsGroupDelay> qzs_gd_ = nullptr;
  std::shared_ptr<GloGroupDelay> glo_gd_ = nullptr;

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace navp::sensors::gnss