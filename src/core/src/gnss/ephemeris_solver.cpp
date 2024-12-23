#include "sensors/gnss/ephemeris_solver.hpp"

#include <ranges>

#include "sensors/gnss/constants.hpp"
#include "sensors/gnss/observation.hpp"
#include "utils/angle.hpp"
#include "utils/logger.hpp"
#include "utils/num_format.hpp"
#include "utils/option.hpp"

using navp::f64;
using navp::utils::NavMatrix33f64;
using navp::utils::NavMatrix34f64;
using navp::utils::NavVector2f64;
using navp::utils::NavVector3f64;
using navp::utils::NavVector4f64;

#define SQR(x) ((x) * (x))
inline f64 constexpr OMGE_GLO = 7.292115E-5;
inline f64 constexpr J2_GLO = 1.0826257E-3;
inline f64 constexpr MU_GLO = 3.9860044E14;
inline f64 constexpr RE_GLO = 6378136.0;

namespace navp::sensors::gnss {

// clang-format off
struct NAVP_EXPORT BdsGroupDelay {
  f64 tgd1=0.0,tgd2=0.0,tgd_b1cp=0.0,tgd_b2ap=0.0,tgd_b2bi=0.0;
  f64 isc_b2ad=0.0,isc_b1cd=0.0;

  void update_d1d2(const Eph* eph) noexcept {
    this->update_d1d2_tgd(eph->tgd);
  }
  void update_cnv1(const Ceph* eph) noexcept {
    this->update_cnv1_tgd_isc(eph->tgd, eph->isc);
  }
  void update_cnv2(const Ceph* eph) noexcept {
    this->update_cnv2_tgd_isc(eph->tgd, eph->isc);
  }
  void update_cnv3(const Ceph* eph) noexcept {
    this->update_cnv3_tgd(eph->tgd);
  }

  void reset_d1d2() noexcept { tgd1=tgd2=0.0; }
  void reset_cnv1() noexcept { tgd_b1cp=tgd_b2ap=isc_b1cd=0.0; }
  void reset_cnv2() noexcept { tgd_b1cp=tgd_b2ap=isc_b2ad=0.0; }
  void reset_cnv3() noexcept { tgd_b2bi=0.0; }
protected:
  constexpr void update_d1d2_tgd(const f64* tgd) noexcept {
    tgd1=tgd[0];tgd2=tgd[1];
  }
  void update_cnv1_tgd_isc(const f64* tgd, const f64* isc) noexcept {
    tgd_b1cp=tgd[0];tgd_b2ap=tgd[1];isc_b1cd=isc[0];
  }
  void update_cnv2_tgd_isc(const f64* tgd, const f64* isc) noexcept {
    tgd_b1cp=tgd[0];tgd_b2ap=tgd[1];isc_b2ad=isc[1];
  }
  void update_cnv3_tgd(const f64* tgd) noexcept { tgd_b2bi=tgd[2]; }
};

struct NAVP_EXPORT GpsGroupDelay {
  f64 tgd=0.0;
  f64 isc_l1ca=0.0,isc_l2c=0.0,isc_l5i5=0.0,isc_l5q5=0.0,isc_l1cd=0.0,isc_l1cp=0.0;

  void update_lnav(const Eph* eph) noexcept {
    update_lnav_tgd(eph->tgd);
  }
  void update_cnav(const Ceph* eph) noexcept {
    update_cnav_tgd_isc(eph->tgd, eph->isc);
  }
  void update_cnv2(const Ceph* eph) noexcept {
    update_cnv2_tgd_isc(eph->tgd, eph->isc);
  }

  void reset_lnav() noexcept { tgd=0.0; }
  void reset_cnav() noexcept { tgd=isc_l1ca=isc_l2c=isc_l5i5=isc_l5q5=0.0; }
  void reset_cnv2() noexcept {
    tgd=isc_l1ca=isc_l2c=isc_l5i5=isc_l5q5=isc_l1cd=isc_l1cp=0.0;
  }
protected:
  void update_lnav_tgd(const f64* _tgd) noexcept { tgd=_tgd[0]; }
  void update_cnav_tgd_isc(const f64* _tgd, const f64* isc) noexcept {
    tgd=_tgd[0];isc_l1ca=isc[0];isc_l2c=isc[1];isc_l5i5=isc[2];isc_l5q5=isc[3];
  }
  void update_cnv2_tgd_isc(const f64* _tgd, const f64* isc) noexcept {
    tgd=_tgd[0];isc_l1ca=isc[0];isc_l2c=isc[1];isc_l5i5=isc[2];
    isc_l5q5=isc[3];isc_l1cd=isc[4];isc_l1cp=isc[5];
  }
};

struct NAVP_EXPORT GalGroupDelay {
  f64 bgd_e5a=0.0, bgd_e5b=0.0;

  void update_ifnav(const Eph* eph) noexcept {
    update_ifnav_tgd(eph->tgd);
  }

  void reset_ifnav() noexcept { bgd_e5a=bgd_e5b=0.0; }

protected:
  void update_ifnav_tgd(const f64* tgd) noexcept { bgd_e5a=tgd[0];bgd_e5b=tgd[1]; }
};

struct NAVP_EXPORT QzsGroupDelay {
  f64 tgd=0.0;
  f64 isc_l1ca=0.0,isc_l2c=0.0,isc_l5i5=0.0,isc_l5q5=0.0,isc_l1cd=0.0,isc_l1cp=0.0;

  void update_lnav(const Eph* eph) noexcept {
    update_lnav_tgd(eph->tgd);
  }
  void update_cnav(const Ceph* eph) noexcept {
    update_cnav_tgd_isc(eph->tgd, eph->isc);
  }
  void update_cnv2(const Ceph* eph) noexcept {
    update_cnv2_tgd_isc(eph->tgd, eph->isc);
  }

  void reset_lnav() noexcept { tgd=0.0; }
  void reset_cnav() noexcept { tgd=isc_l1ca=isc_l2c=isc_l5i5=isc_l5q5=0.0; }
  void reset_cnv2() noexcept {
    tgd=isc_l1ca=isc_l2c=isc_l5i5=isc_l5q5=isc_l1cd=isc_l1cp=0.0;
  }
protected:
  void update_lnav_tgd(const f64* _tgd) noexcept { tgd=_tgd[0]; }
  void update_cnav_tgd_isc(const f64* _tgd, const f64* isc) noexcept {
    tgd=_tgd[0];isc_l1ca=isc[0];isc_l2c=isc[1];isc_l5i5=isc[2];isc_l5q5=isc[3];
  }
  void update_cnv2_tgd_isc(const f64* _tgd, const f64* isc) noexcept {
    tgd=_tgd[0];isc_l1ca=isc[0];isc_l2c=isc[1];isc_l5i5=isc[2];
    isc_l5q5=isc[3];isc_l1cd=isc[4];isc_l1cp=isc[5];
  }
};

struct NAVP_EXPORT GloGroupDelay {
  f64 tgd;

  void update_fdma(const Geph* eph) noexcept {
    update_fdma_tgd(eph->dtaun);
  }

  void reset_fdma() noexcept { this->tgd = 0.0; }

protected:
  void update_fdma_tgd(f64 dtaun) noexcept { this->tgd = dtaun; }
};
// clang-format on

void EphemerisResult::rotate_correct() noexcept {
  if (dt_trans == 0.0) return;
  f64 theta = Constants::omega(sv) * dt_trans;
  f64 sinl = sin(theta), cosl = cos(theta);
  f64 x = cosl * pos[0] + sinl * pos[1];
  f64 y = -sinl * pos[0] + cosl * pos[1];
  pos[0] = x, pos[1] = y;
}

void EphemerisResult::calculate_ea_from(const utils::CoordinateXyz& position) const noexcept {
  auto enu = position.to_enu(pos);
  elevation = asin(sqrt(pow(enu.z(), 2) / enu.squaredNorm()));
  azimuth = atan2(enu.x(), enu.y());
}

std::string EphemerisResult::format_as_string() const noexcept {
  static constexpr auto pos_formatter =
      utils::NumFormatter<utils::AlignmentDirectionEnum::Right, utils::NumTypeEnum::Float, 4 + 10 + 2, 4>();
  static constexpr auto vel_formatter =
      utils::NumFormatter<utils::AlignmentDirectionEnum::Right, utils::NumTypeEnum::Float, 4 + 6 + 2, 4>();
  static constexpr auto dtsv_formatter =
      utils::NumFormatter<utils::AlignmentDirectionEnum::Right, utils::NumTypeEnum::Float, 12 + 4, 12>();

  return std::format("{} {} {} {} {} {} {} {} {}", sv, pos_formatter.format(pos[0]), pos_formatter.format(pos[1]),
                     pos_formatter.format(pos[2]), vel_formatter.format(vel[0]), vel_formatter.format(vel[1]),
                     vel_formatter.format(vel[2]), dtsv_formatter.format(dtsv), dtsv_formatter.format(fd_dtsv));
}

void EphemerisResult::view_vector_to(const utils::CoordinateXyz& station_pos, f64& x, f64& y, f64& z,
                                     f64& distance) const noexcept {
  f64 dx = station_pos.x() - pos.x(), dy = station_pos.y() - pos.y(), dz = station_pos.z() - pos.z();
  distance = sqrt(dx * dx + dy * dy + dz * dz);
  x = dx / distance;
  y = dy / distance;
  z = dz / distance;
}

using utils::GTime;

// static constants
typedef std::unordered_map<ConstellationEnum, std::array<NavMsgTypeEnum, 4>> MsgType;

static MsgType EphMsgTypeMap = {
    {ConstellationEnum::GPS, {NavMsgTypeEnum::LNAV}},
    {ConstellationEnum::BDS, {NavMsgTypeEnum::D1, NavMsgTypeEnum::D2, NavMsgTypeEnum::D1D2}},
    {ConstellationEnum::GAL, {NavMsgTypeEnum::FNAV, NavMsgTypeEnum::INAV, NavMsgTypeEnum::IFNV}},
    {ConstellationEnum::IRN, {NavMsgTypeEnum::LNAV}},
    {ConstellationEnum::QZS, {NavMsgTypeEnum::LNAV}}};

static MsgType CephMsgTypeMap = {
    {ConstellationEnum::GPS, {NavMsgTypeEnum::CNAV, NavMsgTypeEnum::CNV2, NavMsgTypeEnum::CNVX}},
    {ConstellationEnum::QZS, {NavMsgTypeEnum::CNAV, NavMsgTypeEnum::CNV2, NavMsgTypeEnum::CNVX}},
    {ConstellationEnum::BDS, {NavMsgTypeEnum::CNV1, NavMsgTypeEnum::CNV2, NavMsgTypeEnum::CNV3, NavMsgTypeEnum::CNVX}}};

static MsgType GephMsgTypeMap = {{ConstellationEnum::GLO, {NavMsgTypeEnum::FDMA}}};

static MsgType SephMsgTypeMap = {{ConstellationEnum::SBS, {NavMsgTypeEnum::SBAS}}};

struct BrdcKeplerEphHelper;
struct EphSolver;
struct CephSolver;
struct GephSolver;
struct SephSolver;

bool is_eph_vaild(const GTime& t, const GTime& toe, Sv sv) noexcept;
Option<f64> calculate_t_k(const GTime& t, const GTime& toe, Sv sv) noexcept;
bool is_bds_geo(Sv sv) noexcept;

static double var_uraeph(ConstellationEnum sys, i32 ura) {
  constexpr double ura_value[] = {2.4,  3.4,   4.85,  6.85,  9.65,   13.65,  24.0,  48.0,
                                  96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0};
  if (sys == ConstellationEnum::GAL) {
    if (ura <= 49) return SQR(ura * 0.01);
    if (ura <= 74) return SQR(0.5 + (ura - 50) * 0.02);
    if (ura <= 99) return SQR(1.0 + (ura - 75) * 0.04);
    if (ura <= 125) return SQR(2.0 + (ura - 100) * 0.16);
    return SQR(500);
  } else {
    return ura < 0 || 14 < ura ? SQR(6144.0) : SQR(ura_value[ura]);
  }
}

// glonass ephemeris method
static f64 dot(const f64* a, const f64* b, i32 n) {
  f64 c = 0.0;
  while (--n >= 0) c += a[n] * b[n];
  return c;
}
static void deq(const f64* x, f64* xdot, const NavVector3f64& acc) {
  f64 r2 = dot(x, x, 3);
  f64 r3 = r2 * sqrt(r2);
  f64 omg2 = SQR(OMGE_GLO);

  if (r2 <= 0) {
    xdot[0] = 0;
    xdot[1] = 0;
    xdot[2] = 0;
    xdot[3] = 0;
    xdot[4] = 0;
    xdot[5] = 0;

    return;
  }

  /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
  f64 a = 1.5 * J2_GLO * MU_GLO * SQR(RE_GLO) / r2 / r3; /* 3/2*J2*mu*Ae^2/r^5 */
  f64 b = 5.0 * SQR(x[2]) / r2;                          /* 5*z^2/r^2 */
  f64 c = -MU_GLO / r3 - a * (1 - b);                    /* -mu/r^3-a(1-b) */

  xdot[0] = x[3];
  xdot[1] = x[4];
  xdot[2] = x[5];

  xdot[3] = (c + omg2) * x[0] + 2 * OMGE_GLO * x[4] + acc[0];
  xdot[4] = (c + omg2) * x[1] - 2 * OMGE_GLO * x[3] + acc[1];
  xdot[5] = (c - 2 * a) * x[2] + acc[2];
}
static void glorbit(f64 t, f64* x, const NavVector3f64& acc) {
  f64 k1[6];
  f64 k2[6];
  f64 k3[6];
  f64 k4[6];
  f64 w[6];

  deq(x, k1, acc);
  for (i32 i = 0; i < 6; ++i) w[i] = x[i] + k1[i] * t / 2;
  deq(w, k2, acc);
  for (i32 i = 0; i < 6; ++i) w[i] = x[i] + k2[i] * t / 2;
  deq(w, k3, acc);
  for (i32 i = 0; i < 6; ++i) w[i] = x[i] + k3[i] * t;
  deq(w, k4, acc);

  for (i32 i = 0; i < 6; ++i) x[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) * t / 6;
}

struct BrdcKeplerEphHelper {
  Sv sv;                    /// Satellite
  f64 t_k;                  /// The difference between the calculated time and the ephemeris reference time
  f64 u_k;                  /// Ascending angle(corrected)
  f64 r_k;                  /// Radius(corrected)
  f64 i_k;                  /// Orbital inclination(corrected)
  f64 omega_k;              /// Ascending node right ascension
  f64 fd_u_k;               /// First Derivative of Ascending angle(corrected)
  f64 fd_r_k;               /// First Derivative of Radius(corrected)
  f64 fd_i_k;               /// First Derivative of Orbital inclination(corrected)
  f64 fd_omega_k;           /// First Derivative of Ascending node right ascension
  f64 dtsv;                 /// Satellite ranging code phase time offset
  f64 fd_dtsv;              /// Satellite ranging code phase time offset speed
  NavVector2f64 orbit_pos;  /// r_sv in meters kepler orbit

 protected:
  std::tuple<f64, f64> orbit_velocity() const noexcept;
  NavMatrix33f64 meo_orbit_to_ecef_rotation_matrix() const noexcept;
  NavMatrix33f64 geo_orbit_to_ecef_rotation_matrix() const noexcept;
  NavVector3f64 beidou_geo_ecef_position() const noexcept;
  NavVector3f64 beidou_geo_ecef_velocity() const noexcept;
  NavVector3f64 meo_igso_ecef_position() const noexcept;
  std::tuple<NavVector3f64, NavVector3f64> beidou_geo_ecef_position_velocity() const noexcept;
  NavVector3f64 ecef_position() const noexcept;
  NavVector3f64 ecef_velocity() const noexcept;
  std::tuple<NavVector3f64, NavVector3f64> ecef_position_velocity() const noexcept;

 public:
  // calculate sv position
  NavVector3f64 position() const noexcept;

  // calculate sv position and speed
  std::tuple<NavVector3f64, NavVector3f64> position_velocity() const noexcept;
};

struct EphSolver {
  EphSolver(const Navigation::EphMapType* _eph_ptr, Sv sv, const GTime& t) noexcept;

  bool available() const noexcept { return eph; }

  f64 pclk(const GTime& tr) const noexcept;

  EphemerisResult solve(Sv sv, const GTime& tr, const GTime& ts) const noexcept;

 protected:
  const Eph* seleph(Sv sv, const GTime& tr) const noexcept;

  void presolve(Sv sv, const GTime& ts) const noexcept;

  mutable std::shared_ptr<BrdcKeplerEphHelper> helper = nullptr;
  const Navigation::EphMapType* eph_map;
  const Eph* eph;
};

struct CephSolver {
  CephSolver(const Navigation::CephMapType* _eph_ptr, Sv sv, const GTime& t);

  bool available() const noexcept { return eph; }

  f64 pclk(const GTime& tr) const noexcept;

  EphemerisResult solve(Sv sv, const GTime& tr, const GTime& ts) const noexcept;

 protected:
  const Ceph* seleph(Sv sv, const GTime& tr) const noexcept;

  void presolve(Sv sv, const GTime& ts) const noexcept;

  mutable std::shared_ptr<BrdcKeplerEphHelper> helper = nullptr;
  const Navigation::Navigation::CephMapType* eph_map;
  const Ceph* eph;
};

struct GephSolver {
  GephSolver(const Navigation::GephMapType* _eph_ptr, Sv sv, const GTime& t);

  bool available() const noexcept { return eph; }

  f64 pclk(const GTime& tr) const noexcept;

  EphemerisResult solve(Sv sv, const GTime& tr, const GTime& ts) const noexcept;

 protected:
  const Geph* seleph(Sv sv, const GTime& tr) const noexcept;

  const Navigation::GephMapType* eph_map;
  const Geph* eph;
};

struct SephSolver {
  SephSolver(const Navigation::SephMapType* _eph_ptr, Sv sv, const GTime& t);

  bool available() const noexcept { return eph; }

  f64 pclk(const GTime& tr) const noexcept;

  EphemerisResult solve(Sv sv, const GTime& tr, const GTime& ts) const noexcept;

 protected:
  const Seph* seleph(Sv sv, const GTime& tr) const noexcept;

  const Navigation::SephMapType* eph_map;
  const Seph* eph;
};

/*
 * utils function
 */
bool is_eph_vaild(const GTime& t, const GTime& toe, Sv sv) noexcept {
  f64 tk = (f64)((t - toe).to_double());
  return abs(tk) <= Constants::max_toe(sv);
}

Option<f64> calculate_t_k(const GTime& t, const GTime& toe, Sv sv) noexcept {
  f64 tk = (f64)((t - toe).to_double());
  if (abs(tk) <= Constants::max_toe(sv)) {
    return tk;
  } else {
    return None;
  }
}

bool is_bds_geo(Sv sv) noexcept { return sv.system() == ConstellationEnum::BDS && (sv.prn < 6 || sv.prn > 58); }

/*
 * BrdcKeplerEphHelper implement
 */
// tested
std::tuple<f64, f64> BrdcKeplerEphHelper::orbit_velocity() const noexcept {
  auto [_sinuk, _cosuk] = navp::sin_cos(this->u_k);
  auto _fdx = this->fd_r_k * _cosuk - this->r_k * this->fd_u_k * _sinuk;
  auto _fdy = this->fd_r_k * _sinuk + this->r_k * this->fd_u_k * _cosuk;
  return {_fdx, _fdy};
}

// tested
NavMatrix33f64 BrdcKeplerEphHelper::meo_orbit_to_ecef_rotation_matrix() const noexcept {
  using namespace Eigen;
  auto _rotation_x = AngleAxis<f64>(this->i_k, NavVector3f64::UnitX()).toRotationMatrix();
  auto _rotation_z = AngleAxis<f64>(this->omega_k, NavVector3f64::UnitZ()).toRotationMatrix();
  return _rotation_z * _rotation_x;
}

// tested
NavMatrix33f64 BrdcKeplerEphHelper::geo_orbit_to_ecef_rotation_matrix() const noexcept {
  using namespace Eigen;
  auto _rotation_x = AngleAxis<f64>(to_radians(5.0), NavVector3f64::UnitX()).toRotationMatrix();
  auto _rotation_z = AngleAxis<f64>(-Omega::BDS * this->t_k, NavVector3f64::UnitZ()).toRotationMatrix();
  return _rotation_z * _rotation_x;
}

// tested
NavVector3f64 BrdcKeplerEphHelper::beidou_geo_ecef_position() const noexcept {
  using namespace Eigen;
  auto _orbit_xyz = NavVector3f64(this->orbit_pos.x(), this->orbit_pos.y(), 0.0);
  return this->geo_orbit_to_ecef_rotation_matrix() * this->meo_igso_ecef_position();
}

NavVector3f64 BrdcKeplerEphHelper::beidou_geo_ecef_velocity() const noexcept {
  using namespace Eigen;
  auto [_sin_omegak, _cos_omegak] = navp::sin_cos(this->omega_k);
  auto [_sinik, _cosik] = navp::sin_cos(this->i_k);
  auto [_fdx, _fdy] = this->orbit_velocity();

  auto _pos_meo = meo_igso_ecef_position();
  auto _rotation_x = AngleAxis<f64>(to_radians(5.0), NavVector3f64::UnitX()).toRotationMatrix();
  auto _rotation_z = AngleAxis<f64>(-Omega::BDS * this->t_k, NavVector3f64::UnitZ()).toRotationMatrix();
  Matrix<f64, 3, 1> _pos = _rotation_z * _rotation_x * _pos_meo;
  f64 _fd_xgk = -_pos.y() * fd_omega_k - (_fdy * _cosik - _pos.z() * fd_i_k) * _sin_omegak + _fdx * _cos_omegak;
  f64 _fd_ygk = _pos.x() * fd_omega_k + (_fdy * _cosik - _pos.z() * fd_i_k) * _cos_omegak + _fdx * _sin_omegak;
  f64 _fd_zgk = _fdy * _sinik + _pos.y() * fd_i_k * _cosik;
  auto [_sin_omega_tk, _cos_omega_tk] = navp::sin_cos(Omega::BDS * this->t_k);
  auto _fd_rz =
      NavMatrix33f64{{-_sin_omega_tk, _cos_omega_tk, 0.0}, {-_cos_omega_tk, -_sin_omega_tk, 0.0}, {0.0, 0.0, 0.0}};
  _fd_rz = _fd_rz * Omega::BDS;
  auto _fd_pos = NavVector3f64{_fd_xgk, _fd_ygk, _fd_zgk};
  return _fd_rz * _rotation_x * _pos_meo + _rotation_z * _rotation_x * _fd_pos;
}

NavVector3f64 BrdcKeplerEphHelper::meo_igso_ecef_position() const noexcept {
  auto [_sin_omegak, _cos_omegak] = navp::sin_cos(this->omega_k);
  auto [_sinik, _cosik] = navp::sin_cos(this->i_k);
  auto _x = this->orbit_pos.x(), _y = this->orbit_pos.y();
  auto x = _x * _cos_omegak - _y * _cosik * _sin_omegak;
  auto y = _x * _sin_omegak + _y * _cosik * _cos_omegak;
  auto z = _y * _sinik;
  return {x, y, z};
}

std::tuple<NavVector3f64, NavVector3f64> BrdcKeplerEphHelper::beidou_geo_ecef_position_velocity() const noexcept {
  using namespace Eigen;
  // Extract components
  // auto _x = this->orbit_pos.x(), _y = this->orbit_pos.y();
  auto [_sin_omegak, _cos_omegak] = navp::sin_cos(this->omega_k);
  auto [_sinik, _cosik] = navp::sin_cos(this->i_k);
  auto [_fdx, _fdy] = this->orbit_velocity();

  auto _pos_meo = meo_igso_ecef_position();
  auto _rotation_x = AngleAxis<f64>(to_radians(5.0), NavVector3f64::UnitX()).toRotationMatrix();
  auto _rotation_z = AngleAxis<f64>(-Omega::BDS * this->t_k, NavVector3f64::UnitZ()).toRotationMatrix();
  Matrix<f64, 3, 1> _pos = _rotation_z * _rotation_x * _pos_meo;
  f64 _fd_xgk = -_pos.y() * fd_omega_k - (_fdy * _cosik - _pos.z() * fd_i_k) * _sin_omegak + _fdx * _cos_omegak;
  f64 _fd_ygk = _pos.x() * fd_omega_k + (_fdy * _cosik - _pos.z() * fd_i_k) * _cos_omegak + _fdx * _sin_omegak;
  f64 _fd_zgk = _fdy * _sinik + _pos.y() * fd_i_k * _cosik;
  auto [_sin_omega_tk, _cos_omega_tk] = navp::sin_cos(Omega::BDS * this->t_k);
  auto _fd_rz =
      NavMatrix33f64{{-_sin_omega_tk, _cos_omega_tk, 0.0}, {-_cos_omega_tk, -_sin_omega_tk, 0.0}, {0.0, 0.0, 0.0}};
  _fd_rz = _fd_rz * Omega::BDS;
  auto _fd_pos = NavVector3f64{_fd_xgk, _fd_ygk, _fd_zgk};
  Matrix<f64, 3, 1> _vel = _fd_rz * _rotation_x * _pos_meo + _rotation_z * _rotation_x * _fd_pos;
  return {_pos, _vel};
}

// tested
NavVector3f64 BrdcKeplerEphHelper::ecef_position() const noexcept {
  if (is_bds_geo(this->sv)) {
    return this->beidou_geo_ecef_position();
  } else {
    return this->meo_igso_ecef_position();
  }
}

NavVector3f64 BrdcKeplerEphHelper::ecef_velocity() const noexcept {
  if (is_bds_geo(this->sv)) {
    return this->beidou_geo_ecef_velocity();
  } else {
    auto _x = this->orbit_pos.x(), _y = this->orbit_pos.y();
    auto [_sin_omegak, _cos_omegak] = navp::sin_cos(this->omega_k);
    auto [_sinik, _cosik] = navp::sin_cos(this->i_k);
    auto [_fdx, _fdy] = this->orbit_velocity();
    NavMatrix34f64 _fdr;
    _fdr(0, 0) = _cos_omegak;
    _fdr(0, 1) = -_sin_omegak * _cosik;
    _fdr(0, 2) = -(_x * _sin_omegak + _y * _cos_omegak * _cosik);
    _fdr(0, 3) = _y * _sin_omegak * _sinik;
    _fdr(1, 0) = _sin_omegak;
    _fdr(1, 1) = _cos_omegak * _cosik;
    _fdr(1, 2) = _x * _cos_omegak - _y * _sin_omegak * _cosik;
    _fdr(1, 3) = _y * _cos_omegak * _sinik;
    _fdr(2, 0) = 0.0;
    _fdr(2, 1) = _sinik;
    _fdr(2, 2) = 0.0;
    _fdr(2, 3) = _y * _cosik;

    auto _rhs = NavVector4f64{_fdx, _fdy, this->fd_omega_k, this->fd_i_k};
    return _fdr * _rhs;
  }
}

std::tuple<NavVector3f64, NavVector3f64> BrdcKeplerEphHelper::ecef_position_velocity() const noexcept {
  if (is_bds_geo(sv)) {
    return this->beidou_geo_ecef_position_velocity();
  } else {
    return {this->ecef_position(), this->ecef_velocity()};
  }
}

NavVector3f64 BrdcKeplerEphHelper::position() const noexcept { return this->ecef_position(); }

std::tuple<NavVector3f64, NavVector3f64> BrdcKeplerEphHelper::position_velocity() const noexcept {
  return this->ecef_position_velocity();
}

/*
 * EphSolver implementation
 */
EphSolver::EphSolver(const Navigation::EphMapType* _eph_ptr, Sv sv, const GTime& t) noexcept {
  this->eph_map = _eph_ptr;
  this->eph = this->seleph(sv, t);
}

const Eph* EphSolver::seleph(Sv sv, const GTime& t) const noexcept {
  if (!this->eph_map->contains(sv)) {
    return nullptr;
  }
  auto _eph_map = this->eph_map->at(sv);
  for (const auto& nav_msg_type : EphMsgTypeMap.at(sv.system())) {
    if (_eph_map.contains(nav_msg_type)) {
      for (const auto& [toe, _ephemeris] : _eph_map.at(nav_msg_type)) {
        if (abs((t - toe).to_double()) <= Constants::max_toe(sv)) {
          return &_ephemeris;
        }
      }
    }
  }
  return nullptr;
}

void EphSolver::presolve(Sv sv, const GTime& t) const noexcept {
  if (!this->eph) {
    return;
  }
  const auto* _eph = this->eph;

  // constants
  auto _gm = Constants::gm(sv);
  auto _omega = Constants::omega(sv);
  auto _dtr_f = Constants::dtr_f(sv);
  // tk

  auto _tk_op = calculate_t_k(t, _eph->toe, sv);
  if (_tk_op.is_none()) return;
  auto _tk = _tk_op.unwrap_unchecked();

  auto _n0 = sqrt((_gm / pow(_eph->A, 3.0)));  // average angular velocity
  auto _n = _n0 + _eph->deln;                  // corrected mean angular velocity
  auto _mk = _eph->M0 + _n * _tk;              // average anomaly

  // Iterative calculation of e_k
  f64 _ek0 = 0.0, _ek = 0.0;
  u8 i = 0;
  while (true) {
    _ek = _mk + _eph->e * sin(_ek0);
    if (abs(_ek - _ek0) < 1e-10) {
      break;
    }
    _ek0 = _ek;
    ++i;
  }
  if (i >= MaxIterNumber::KEPLER) {
    nav_error("{} kepler iteration overflow", std::format("{}", sv));
    return;
  }

  // true anomaly
  auto [_sinek, _cosek] = navp::sin_cos(_ek);
  auto _vk = atan2(sqrt(1.0 - pow(_eph->e, 2.0)) * _sinek, _cosek - _eph->e);

  auto _phik = _vk + _eph->omg;
  auto [_sin2phik, _cos2phik] = navp::sin_cos(2 * _phik);

  // latitude argument correction
  auto _duk = _eph->cus * _sin2phik + _eph->cuc * _cos2phik;
  auto _uk = _phik + _duk;

  // orbital radisu correction
  auto _drk = _eph->crs * _sin2phik + _eph->crc * _cos2phik;
  auto _rk = _eph->A * (1.0 - _eph->e * _cosek) + _drk;

  // inclination angle correction
  auto _dik = _eph->cis * _sin2phik + _eph->cic * _cos2phik;
  auto _ik = _eph->i0 + _dik + _eph->idot * _tk;

  // first derivatives, needed to calculate speed of Sv
  f64 _fd_omegak = 0.0;
  if (is_bds_geo(sv)) {
    _fd_omegak = _eph->OMGd;
  } else {
    _fd_omegak = _eph->OMGd - _omega;
  }
  auto _fd_ek = _n / (1.0 - _eph->e * _cosek);
  auto _fd_phi_k = sqrt((1.0 + _eph->e) / (1.0 - _eph->e)) * pow((cos(_vk / 2.0) / cos(_ek / 2.0)), 2) * _fd_ek;
  auto _fd_uk = (_eph->cus * _cos2phik - _eph->cuc * _sin2phik) * _fd_phi_k * 2.0 + _fd_phi_k;
  auto _fd_rk = _eph->A * _eph->e * _sinek * _fd_ek + 2.0 * (_eph->crs * _cos2phik - _eph->crc * _sin2phik) * _fd_phi_k;
  auto _fd_ik = _eph->idot + 2.0 * (_eph->cis * _cos2phik - _eph->cic * _sin2phik) * _fd_phi_k;

  // relativistic effect correction
  auto _dtr = _dtr_f * _eph->e * sqrt(_eph->A) * _sinek;
  auto _fd_dtr = _dtr_f * _eph->e * sqrt(_eph->A) * _cosek * _fd_ek;
  // calculate dtsv
  auto dt = (f64)((t - _eph->toc).to_double());
  auto _dtsv = _eph->f0 + _eph->f1 * dt + _eph->f2 * dt * dt + _dtr;
  auto _fd_dtsv = _eph->f1 + 2 * _eph->f2 * dt + _fd_dtr;

  // ascending node longitude correction
  f64 _omegak;
  if (is_bds_geo(sv)) {
    // BeiDou [GEO]
    _omegak = _eph->OMG0 + _eph->OMGd * _tk - _omega * _eph->toes;
  } else {
    // GPS, Galileo, BeiDou [MEO,IGSO]
    _omegak = _eph->OMG0 + (_eph->OMGd - _omega) * _tk - _omega * _eph->toes;
  }

  // position in kepler orbit plane
  auto x = _rk * cos(_uk), y = _rk * sin(_uk);

  this->helper = std::make_shared<BrdcKeplerEphHelper>(BrdcKeplerEphHelper{.sv = sv,
                                                                           .t_k = _tk,
                                                                           .u_k = _uk,
                                                                           .r_k = _rk,
                                                                           .i_k = _ik,
                                                                           .omega_k = _omegak,
                                                                           .fd_u_k = _fd_uk,
                                                                           .fd_r_k = _fd_rk,
                                                                           .fd_i_k = _fd_ik,
                                                                           .fd_omega_k = _fd_omegak,
                                                                           .dtsv = _dtsv,
                                                                           .fd_dtsv = _fd_dtsv,
                                                                           .orbit_pos = {x, y}});
}

f64 EphSolver::pclk(const GTime& tr) const noexcept {
  f64 t, ts;
  t = ts = (tr - eph->toc).to_double();
  for (int i = 0; i < 2; ++i) {
    t = ts - (eph->f0 + eph->f1 * t + eph->f2 * t * t);
  }
  return eph->f0 + eph->f1 * t + eph->f2 * t * t;
}

EphemerisResult EphSolver::solve(Sv sv, const GTime& tr, const GTime& ts) const noexcept {
  presolve(sv, ts);
  EphemerisResult result;
  auto [pos, vel] = this->helper->position_velocity();
  result.sv = sv;
  result.pos = pos;
  result.vel = vel;
  result.dt_trans = (tr - ts).to_double();
  result.dtsv = helper->dtsv;
  result.fd_dtsv = helper->fd_dtsv;
  result.var = var_uraeph(sv.system(), eph->sva);
  // correct rotation
  result.rotate_correct();
  return result;
}

/*
 * CephSolver implementation
 */

CephSolver::CephSolver(const Navigation::Navigation::CephMapType* _eph_ptr, Sv sv, const GTime& t) {
  this->eph_map = _eph_ptr;
  this->eph = this->seleph(sv, t);
}

const Ceph* CephSolver::seleph(Sv sv, const GTime& t) const noexcept {
  if (!this->eph_map->contains(sv)) {
    return nullptr;
  }
  auto _eph_map = this->eph_map->at(sv);
  for (const auto& nav_msg_type : CephMsgTypeMap.at(sv.system())) {
    if (_eph_map.contains(nav_msg_type)) {
      for (const auto& [toe, _ephemeris] : _eph_map.at(nav_msg_type)) {
        if (abs((t - toe).to_double()) <= Constants::max_toe(sv)) {
          return &_ephemeris;
        }
      }
    }
  }
  return nullptr;
}

void CephSolver::presolve(Sv sv, const GTime& t) const noexcept {
  if (!this->eph) {
    return;
  }
  const auto* _eph = this->eph;

  // constants
  auto _gm = Constants::gm(sv);
  auto _omega = Constants::omega(sv);
  auto _dtr_f = Constants::dtr_f(sv);
  // tk
  auto _tk_op = calculate_t_k(t, _eph->toe, sv);
  if (_tk_op.is_none()) return;
  auto _tk = _tk_op.unwrap_unchecked();

  // A
  auto _a = _eph->A + _eph->Adot * _tk;
  auto _n0 = sqrt((_gm / pow(_eph->A, 3.0)));           // average angular velocity
  auto _n = _n0 + _eph->deln + 0.5 * _eph->dn0d * _tk;  // corrected mean angular velocity
  auto _mk = _eph->M0 + _n * _tk;                       // average anomaly

  // Iterative calculation of e_k
  f64 _ek0 = 0, _ek = 1;
  u8 i;
  while (abs(_ek - _ek0) >= 1e-10) {
    _ek = _mk + _eph->e * sin(_ek0);
    _ek0 = _ek;
    ++i;
  }
  if (i >= MaxIterNumber::KEPLER) {
    nav_error("{} kepler iteration overflow", std::format("{}", sv));
    return;
  }

  // true anomaly
  auto [_sinek, _cosek] = navp::sin_cos(_ek);
  auto _vk = atan2(sqrt(1.0 - pow(_eph->e, 2.0)) * _sinek, _cosek - _eph->e);

  auto _phik = _vk + _eph->omg;
  auto [_sin2phik, _cos2phik] = navp::sin_cos(2 * _phik);

  // latitude argument correction
  auto _duk = _eph->cus * _sin2phik + _eph->cuc * _cos2phik;
  auto _uk = _phik + _duk;

  // orbital radisu correction
  auto _drk = _eph->crs * _sin2phik + _eph->crc * _cos2phik;
  auto _rk = _a * (1.0 - _eph->e * _cosek) + _drk;

  // inclination angle correction
  auto _dik = _eph->cis * _sin2phik + _eph->cic * _cos2phik;
  auto _ik = _eph->i0 + _dik + _eph->idot * _tk;

  // first derivatives, needed to calculate speed of Sv
  auto _fd_omegak = _eph->OMGd - _omega;
  auto _fd_ek = _n / (1.0 - _eph->e * _cosek);
  auto _fd_phi_k = sqrt((1.0 + _eph->e) / (1.0 - _eph->e)) * pow((cos(_vk / 2.0) / cos(_ek / 2.0)), 2) * _fd_ek;
  auto _fd_uk = (_eph->cus * _cos2phik - _eph->cuc * _sin2phik) * _fd_phi_k * 2.0 + _fd_phi_k;
  auto _fd_rk = _a * _eph->e * _sinek * _fd_ek + 2.0 * (_eph->crs * _cos2phik - _eph->crc * _sin2phik) * _fd_phi_k;
  auto _fd_ik = _eph->idot + 2.0 * (_eph->cis * _cos2phik - _eph->cic * _sin2phik) * _fd_phi_k;

  // relativistic effect correction
  auto _dtr = _dtr_f * _eph->e * sqrt(_a) * _sinek;
  auto _fd_dtr = _dtr_f * _eph->e * sqrt(_a) * _cosek * _fd_ek;
  auto dt = (f64)((t - _eph->toc).to_double());
  auto _dtsv = _eph->f0 + _eph->f1 * dt + _eph->f2 * dt * dt + _dtr;
  auto _fd_dtsv = _eph->f1 + 2 * _eph->f2 * dt + _fd_dtr;

  // ascending node longitude correction
  f64 _omegak;
  if (is_bds_geo(sv)) {
    // BeiDou [GEO]
    _omegak = _eph->OMG0 + _eph->OMGd * _tk - _omega * _eph->toes;
  } else {
    // GPS, Galileo, BeiDou [MEO,IGSO]
    _omegak = _eph->OMG0 + (_eph->OMGd - _omega) * _tk - _omega * _eph->toes;
  }

  // position in kepler orbit plane
  auto x = _rk * cos(_uk), y = _rk * sin(_uk);

  this->helper = std::make_shared<BrdcKeplerEphHelper>(BrdcKeplerEphHelper{.sv = sv,
                                                                           .t_k = _tk,
                                                                           .u_k = _uk,
                                                                           .r_k = _rk,
                                                                           .i_k = _ik,
                                                                           .omega_k = _omegak,
                                                                           .fd_u_k = _fd_uk,
                                                                           .fd_r_k = _fd_rk,
                                                                           .fd_i_k = _fd_ik,
                                                                           .fd_omega_k = _fd_omegak,
                                                                           .dtsv = _dtsv,
                                                                           .fd_dtsv = _fd_dtsv,
                                                                           .orbit_pos = {x, y}});
}

f64 CephSolver::pclk(const GTime& tr) const noexcept {
  f64 t, ts;
  t = ts = (tr - eph->toc).to_double();
  for (int i = 0; i < 2; ++i) {
    t = ts - (eph->f0 + eph->f1 * t + eph->f2 * t * t);
  }
  return eph->f0 + eph->f1 * t + eph->f2 * t * t;
}

EphemerisResult CephSolver::solve(Sv sv, const GTime& tr, const GTime& ts) const noexcept {
  presolve(sv, ts);
  EphemerisResult result;
  auto [pos, vel] = this->helper->position_velocity();
  result.sv = sv;
  result.pos = pos;
  result.vel = vel;
  // todo
  // sis
  // result.var = var_uraeph(sv.system(), eph);
  result.dt_trans = (tr - ts).to_double();
  result.dtsv = helper->dtsv;
  result.fd_dtsv = helper->fd_dtsv;
  // correct rotation
  result.rotate_correct();
  return result;
}

/*
 * GephSolver implementation
 */
GephSolver::GephSolver(const Navigation::GephMapType* _eph_ptr, Sv sv, const GTime& t) {
  this->eph_map = _eph_ptr;
  this->eph = this->seleph(sv, t);
}

const Geph* GephSolver::seleph(Sv sv, const GTime& t) const noexcept {
  if (!this->eph_map->contains(sv)) {
    return nullptr;
  }
  auto _eph_map = this->eph_map->at(sv);
  for (const auto& nav_msg_type : GephMsgTypeMap.at(sv.system())) {
    if (_eph_map.contains(nav_msg_type)) {
      for (const auto& [toe, _ephemeris] : _eph_map.at(nav_msg_type)) {
        if (abs((t - toe).to_double()) <= Constants::max_toe(sv)) {
          return &_ephemeris;
        }
      }
    }
  }
  return nullptr;
}

f64 GephSolver::pclk(const GTime& tr) const noexcept {
  f64 t = (tr - eph->toe).to_double(), ts = t;
  for (i32 i = 0; i < 2; ++i) {
    t = ts - (-eph->taun + eph->gammaN * t);
  }
  return -eph->taun + eph->gammaN * t;
}

EphemerisResult GephSolver::solve(Sv sv, const GTime& tr, const GTime& ts) const noexcept {
  EphemerisResult result;
  result.sv = sv;
  result.dt_trans = (tr - ts).to_double();

  f64 t = (ts - eph->toe).to_double();
  result.dtsv = -eph->taun + eph->gammaN * t;
  result.fd_dtsv = eph->gammaN;

  f64 x[6];
  for (i32 i = 0; i < 3; ++i) {
    x[i] = eph->pos[i];
    x[i + 3] = eph->vel[i];
  }
  for (f64 tt = t < 0 ? -60.0 : 60.0; fabs(t) > 1E-9; t -= tt) {
    if (fabs(t) < 60.0) tt = t;
    glorbit(tt, x, eph->acc);
  }
  for (i32 i = 0; i < 3; ++i) {
    result.pos[i] = x[i];
  }
  for (i32 i = 3; i < 6; ++i) {
    result.vel[i - 3] = x[i];
  }
  result.var = var_uraeph(sv.system(), eph->sva);
  return result;
}

/*
 * SephSolver implementation
 */

SephSolver::SephSolver(const Navigation::SephMapType* _eph_ptr, Sv sv, const GTime& t) {
  this->eph_map = _eph_ptr;
  this->eph = this->seleph(sv, t);
}

const Seph* SephSolver::seleph(Sv sv, const GTime& t) const noexcept {
  if (!this->eph_map->contains(sv)) {
    return nullptr;
  }
  auto _eph_map = this->eph_map->at(sv);
  for (const auto& nav_msg_type : SephMsgTypeMap.at(sv.system())) {
    if (_eph_map.contains(nav_msg_type)) {
      for (const auto& [toe, _ephemeris] : _eph_map.at(nav_msg_type)) {
        if (abs((t - toe).to_double()) <= Constants::max_toe(sv)) {
          return &_ephemeris;
        }
      }
    }
  }
  return nullptr;
}

f64 SephSolver::pclk(const GTime& tr) const noexcept {
  f64 t = (tr - eph->t0).to_double();
  for (i32 i = 0; i < 2; ++i) {
    t -= eph->af0 + eph->af1 * t;
  }
  return eph->af0 + eph->af1 * t;
}

EphemerisResult SephSolver::solve(Sv sv, const GTime& tr, const GTime& ts) const noexcept {
  EphemerisResult result;
  result.sv = sv;
  result.dt_trans = (tr - ts).to_double();

  f64 t = (ts - eph->t0).to_double();
  result.dtsv = eph->af0 + eph->af1 * t;
  result.fd_dtsv = eph->af1;

  for (i32 i = 0; i < 3; ++i) {
    result.pos[i] = eph->pos[i] + eph->vel[i] * t + eph->acc[i] * t * t * 0.5;
  }
  for (i32 i = 0; i < 3; ++i) {
    result.vel[i] = eph->vel[i] + eph->acc[i] * t;
  }
  result.var = var_uraeph(sv.system(), eph->sva);
  return result;
}

/*
 * EphemerisSolver implementation
 */
EphemerisSolver::EphemerisSolver(std::shared_ptr<spdlog::logger> logger) noexcept
    : sv_status_(std::make_shared<TimeSvMap>()),
      bds_gd_(std::make_shared<BdsGroupDelay>()),
      gps_gd_(std::make_shared<GpsGroupDelay>()),
      gal_gd_(std::make_shared<GalGroupDelay>()),
      qzs_gd_(std::make_shared<QzsGroupDelay>()),
      glo_gd_(std::make_shared<GloGroupDelay>()),
      logger_(logger) {}

bool update_newest_eph(Eph* _cur_eph, const utils::GTime& t, const Navigation* nav, ConstellationEnum cons,
                       NavMsgTypeEnum nav_msg_type) noexcept {
  if (_cur_eph && is_eph_vaild(t, _cur_eph->toe, Sv{0, cons})) {
    return true;
  } else {
    if (!nav) return false;  // check poiter vaild
    for (const auto& [_sv, _map] : nav->ephMap) {
      for (const auto& [_nav_msg_type, _eph_map] : _map) {
        if (_nav_msg_type == nav_msg_type) {
          for (const auto& [_t, _eph] : _eph_map) {
            if (is_eph_vaild(t, _eph.toe, Sv{0, cons})) {
              _cur_eph = const_cast<Eph*>(std::addressof(_eph));
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

bool update_newest_ceph(Ceph* _cur_eph, const utils::GTime& t, const Navigation* nav, ConstellationEnum cons,
                        NavMsgTypeEnum nav_msg_type) noexcept {
  if (_cur_eph && is_eph_vaild(t, _cur_eph->toe, Sv{0, cons})) {
    return true;
  } else {
    if (!nav) return false;  // check poiter vaild
    for (const auto& [_sv, _map] : nav->cephMap) {
      for (const auto& [_nav_msg_type, _eph_map] : _map) {
        if (_nav_msg_type == nav_msg_type) {
          for (const auto& [_t, _ceph] : _eph_map) {
            if (is_eph_vaild(t, _ceph.toe, Sv{0, cons})) {
              _cur_eph = const_cast<Ceph*>(std::addressof(_ceph));
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

bool update_newest_geph(Geph* _cur_geph, const utils::GTime& t, const Navigation* nav,
                        NavMsgTypeEnum nav_msg_type) noexcept {
  if (_cur_geph && is_eph_vaild(t, _cur_geph->toe, Sv{0, ConstellationEnum::GLO})) {
    return true;
  } else {
    if (!nav) return false;  // check poiter vaild
    for (const auto& [_sv, _map] : nav->gephMap) {
      for (const auto& [_nav_msg_type, _eph_map] : _map) {
        if (_nav_msg_type == nav_msg_type) {
          for (const auto& [_t, _ceph] : _eph_map) {
            if (is_eph_vaild(t, _ceph.toe, Sv{0, ConstellationEnum::GLO})) {
              _cur_geph = const_cast<Geph*>(std::addressof(_ceph));
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

bool EphemerisSolver::update_newest_gps_lnav(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_eph(cache_gps_lnav_, t, _nav, ConstellationEnum::GPS, NavMsgTypeEnum::LNAV);
  });
}

bool EphemerisSolver::update_newest_gps_cnav(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_ceph(cache_gps_cnav_, t, _nav, ConstellationEnum::GPS, NavMsgTypeEnum::CNAV);
  });
}

bool EphemerisSolver::update_newest_gps_cnv2(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_ceph(cache_gps_cnv2_, t, _nav, ConstellationEnum::GPS, NavMsgTypeEnum::CNV2);
  });
}

bool EphemerisSolver::update_newest_bds_d1d2(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_eph(cache_bds_d1d2_, t, _nav, ConstellationEnum::BDS, NavMsgTypeEnum::D1D2) ||
           update_newest_eph(cache_bds_d1d2_, t, _nav, ConstellationEnum::BDS, NavMsgTypeEnum::D1) ||
           update_newest_eph(cache_bds_d1d2_, t, _nav, ConstellationEnum::BDS, NavMsgTypeEnum::D2);
  });
}

bool EphemerisSolver::update_newest_bds_cnv1(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_ceph(cache_bds_cnv1_, t, _nav, ConstellationEnum::BDS, NavMsgTypeEnum::CNV1);
  });
}

bool EphemerisSolver::update_newest_bds_cnv2(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_ceph(cache_bds_cnv2_, t, _nav, ConstellationEnum::BDS, NavMsgTypeEnum::CNV2);
  });
}

bool EphemerisSolver::update_newest_bds_cnv3(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_ceph(cache_bds_cnv3_, t, _nav, ConstellationEnum::BDS, NavMsgTypeEnum::CNV3);
  });
}

bool EphemerisSolver::update_newest_gal_ifnav(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_eph(cache_gal_ifnav_, t, _nav, ConstellationEnum::GAL, NavMsgTypeEnum::INAV) ||
           update_newest_eph(cache_gal_ifnav_, t, _nav, ConstellationEnum::GAL, NavMsgTypeEnum::FNAV) ||
           update_newest_eph(cache_gal_ifnav_, t, _nav, ConstellationEnum::GAL, NavMsgTypeEnum::IFNV);
  });
}

bool EphemerisSolver::update_newest_qzs_lnav(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_eph(cache_qzs_lnav_, t, _nav, ConstellationEnum::QZS, NavMsgTypeEnum::LNAV);
  });
}

bool EphemerisSolver::update_newest_qzs_cnav(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_ceph(cache_qzs_cnav_, t, _nav, ConstellationEnum::QZS, NavMsgTypeEnum::CNAV);
  });
}

bool EphemerisSolver::update_newest_qzs_cnv2(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_ceph(cache_qzs_cnv2_, t, _nav, ConstellationEnum::QZS, NavMsgTypeEnum::CNV2);
  });
}

bool EphemerisSolver::update_newest_glo_fdma(const utils::GTime& t) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    return update_newest_geph(cache_glo_fdma_, t, _nav, NavMsgTypeEnum::FDMA);
  });
}

bool EphemerisSolver::launch_ceph_solver(const utils::GTime& tr, Sv _sv, f64 pr, bool correct_transmission) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    if (!_nav) return false;  // check pointer vaild
    CephSolver _ceph_solver(&_nav->cephMap, _sv, tr);
    if (_ceph_solver.available()) {
      auto ts = tr;
      if (correct_transmission) {
        ts.bigTime -= pr / Constants::CLIGHT;
        ts.bigTime -= _ceph_solver.pclk(ts);
      }
      (*sv_status_)[static_cast<EpochUtc>(tr)][_sv] = _ceph_solver.solve(_sv, tr, ts);
      return true;
    }
    return false;
  });
}

bool EphemerisSolver::launch_eph_solver(const utils::GTime& tr, Sv _sv, f64 pr, bool correct_transmission) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    if (!_nav) return false;  // check pointer vaild
    EphSolver _eph_solver(&_nav->ephMap, _sv, tr);
    if (_eph_solver.available()) {
      auto ts = tr;
      if (correct_transmission) {
        ts.bigTime -= pr / Constants::CLIGHT;
        ts.bigTime -= _eph_solver.pclk(ts);
      }
      (*sv_status_)[static_cast<EpochUtc>(tr)][_sv] = _eph_solver.solve(_sv, tr, ts);
      return true;
    }
    return false;
  });
}

bool EphemerisSolver::launch_geph_solver(const utils::GTime& tr, Sv _sv, f64 pr, bool correct_transmission) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    if (!_nav) return false;  // check pointer vaild
    GephSolver _geph_solver(&_nav->gephMap, _sv, tr);
    if (_geph_solver.available()) {
      auto ts = tr;
      if (correct_transmission) {
        ts.bigTime -= pr / Constants::CLIGHT;
        ts.bigTime -= _geph_solver.pclk(ts);
      }
      (*sv_status_)[static_cast<EpochUtc>(tr)][_sv] = _geph_solver.solve(_sv, tr, ts);
      return true;
    }
    return false;
  });
}

bool EphemerisSolver::launch_seph_solver(const utils::GTime& tr, Sv _sv, f64 pr, bool correct_transmission) noexcept {
  return std::ranges::any_of(nav_vec_, [&](const Navigation* _nav) {
    if (!_nav) return false;  // check pointer vaild
    SephSolver _seph_solver(&_nav->sephMap, _sv, tr);
    if (_seph_solver.available()) {
      auto ts = tr;
      if (correct_transmission) {
        ts.bigTime -= pr / Constants::CLIGHT;
        ts.bigTime -= _seph_solver.pclk(ts);
      }
      (*sv_status_)[static_cast<EpochUtc>(tr)][_sv] = _seph_solver.solve(_sv, tr, ts);
      return true;
    }
    return false;
  });
}

void EphemerisSolver::add_ephemeris(const Navigation* _nav) noexcept { nav_vec_.emplace_back(_nav); }

auto EphemerisSolver::solve_sv_status(EpochUtc tr,
                                      const GnssObsRecord::ObsMap* visible_sv) noexcept -> std::vector<Sv> {
  std::ranges::for_each(*visible_sv, [&](const auto& kv) {
    Sv sv = kv.first;
    const GObs& obs = *kv.second;

    f64 pr = 0.0;
    for (const auto& sig : obs.sigs_list | std::views::values | std::views::join) {
      if (sig.pseudorange != 0.0) {
        pr = sig.pseudorange;
        break;
      }
    }
    if (pr == 0.0) return;

    if (brdc_solve_sv_status(tr, sv, pr)) {
      logger_->debug("solve sv status: {}", sv);
    }
  });
  return sv_status_->at(tr) | std::views::keys | std::ranges::to<std::vector>();
}

std::vector<Sv> EphemerisSolver::solve_sv_status(EpochUtc tr, const std::vector<Sv>& sv) noexcept {
  for (auto _sv : sv) {
    brdc_solve_sv_status(tr, _sv, 0.0);
  }
  return sv_status_->at(tr) | std::views::keys | std::ranges::to<std::vector>();
}

bool EphemerisSolver::brdc_solve_sv_status(EpochUtc tr, Sv _sv, f64 pr) noexcept {
  bool correct_transmission = pr == 0.0 ? false : true;
  bool done = false;
  auto _cons = _sv.system();
  if (_cons == ConstellationEnum::GPS || _cons == ConstellationEnum::BDS || _cons == ConstellationEnum::QZS) {
    done = launch_ceph_solver(tr, _sv, pr, correct_transmission);
    if (!done) launch_eph_solver(tr, _sv, pr, correct_transmission);
  } else if (_cons == ConstellationEnum::GAL) {
    done = launch_eph_solver(tr, _sv, pr, correct_transmission);
  } else if (_cons == ConstellationEnum::GLO) {
    done = launch_geph_solver(tr, _sv, pr, correct_transmission);
  } else if (_cons == ConstellationEnum::SBS) {
    done = launch_seph_solver(tr, _sv, pr, correct_transmission);
  } else {
    nav_error("unsupport constellation {}", magic_enum::enum_name(_cons))
  }
  return done;
}

void EphemerisSolver::update_tgd(ConstellationEnum sys, const utils::GTime& t) noexcept {
  switch (sys) {
    case ConstellationEnum::BDS: {
      // update d1d2
      if (update_newest_bds_d1d2(t)) {
        bds_gd_->update_d1d2(cache_bds_d1d2_);
      } else {
        bds_gd_->reset_d1d2();
      }
      // update cnv1
      if (update_newest_bds_cnv1(t)) {
        bds_gd_->update_cnv1(cache_bds_cnv1_);
      } else {
        bds_gd_->reset_cnv1();
      }
      // update cnv2
      if (update_newest_bds_cnv2(t)) {
        bds_gd_->update_cnv2(cache_bds_cnv2_);
      } else {
        bds_gd_->reset_cnv2();
      }
      // update cnv3
      if (update_newest_bds_cnv3(t)) {
        bds_gd_->update_cnv3(cache_bds_cnv3_);
      } else {
        bds_gd_->reset_cnv3();
      }
    }
    case ConstellationEnum::GPS: {
      // update lnav
      if (update_newest_gps_lnav(t)) {
        gps_gd_->update_lnav(cache_gps_lnav_);
      } else {
        gps_gd_->reset_lnav();
      }
      // update cnav
      if (update_newest_gps_cnav(t)) {
        gps_gd_->update_cnav(cache_gps_cnav_);
      } else {
        gps_gd_->reset_cnav();
      }
      // update cnv2
      if (update_newest_gps_cnv2(t)) {
        gps_gd_->update_cnv2(cache_gps_cnv2_);
      } else {
        gps_gd_->reset_cnv2();
      }
    }
    case ConstellationEnum::GAL: {
      // update ifnav
      if (update_newest_gal_ifnav(t)) {
        gal_gd_->update_ifnav(cache_gal_ifnav_);
      } else {
        gal_gd_->reset_ifnav();
      }
    }
    case ConstellationEnum::QZS: {
      // update lnav
      if (update_newest_qzs_lnav(t)) {
        qzs_gd_->update_lnav(cache_qzs_lnav_);
      } else {
        qzs_gd_->reset_lnav();
      }
      // update cnav
      if (update_newest_qzs_cnav(t)) {
        qzs_gd_->update_cnav(cache_qzs_cnav_);
      } else {
        qzs_gd_->reset_cnav();
      }
      // update cnv2
      if (update_newest_qzs_cnv2(t)) {
        qzs_gd_->update_cnv2(cache_qzs_cnv2_);
      } else {
        qzs_gd_->reset_cnv2();
      }
    }
    case ConstellationEnum::GLO: {
      if (update_newest_glo_fdma(t)) {
        glo_gd_->update_fdma(cache_glo_fdma_);
      } else {
        glo_gd_->reset_fdma();
      }
    }
    default: {
      return;
    }
  }
}

auto EphemerisSolver::quary_sv_status(EpochUtc t) const noexcept -> const SvMap* {
  if (sv_status_->contains(t)) {
    return &sv_status_->at(t);
  } else {
    return nullptr;
  }
}

auto EphemerisSolver::quary_sv_status(EpochUtc t, Sv sv) const noexcept -> const EphemerisResult* {
  if (!sv_status_->contains(t) && !sv_status_->at(t).contains(sv)) {
    return nullptr;
  } else {
    return &sv_status_->at(t).at(sv);
  }
}

std::vector<const EphemerisResult*> EphemerisSolver::quary_sv_status(EpochUtc t,
                                                                     const std::vector<Sv>& sv) const noexcept {
  if (!sv_status_->contains(t)) {
    return {};
  }
  std::vector<const EphemerisResult*> res(sv.size());
  const auto& sv_map = sv_status_->at(t);
  for (u16 i = 0; i < sv.size(); ++i) {
    res[i] = &sv_map.at(sv[i]);
  }
  return std::move(res);
}

std::vector<const EphemerisResult*> EphemerisSolver::quary_sv_status_unchecked(EpochUtc t,
                                                                               const std::vector<Sv>& sv) const {
  std::vector<const EphemerisResult*> res(sv.size());
  const auto& sv_map = sv_status_->at(t);
  for (u16 i = 0; i < sv.size(); ++i) {
    res[i] = &sv_map.at(sv[i]);
  }
  return std::move(res);
}

auto EphemerisSolver::quary_sv_status_unchecked(EpochUtc t, Sv sv) const -> const EphemerisResult* {
  return &sv_status_->at(t).at(sv);
}

auto EphemerisSolver::quary_gps_tgd(EpochUtc t) noexcept -> const GpsGroupDelay* {
  update_tgd(ConstellationEnum::GPS, static_cast<GTime>(t));
  return this->gps_gd_.get();
}

auto EphemerisSolver::quary_bds_tgd(EpochUtc t) noexcept -> const BdsGroupDelay* {
  update_tgd(ConstellationEnum::BDS, static_cast<GTime>(t));
  return this->bds_gd_.get();
}

auto EphemerisSolver::quary_gal_tgd(EpochUtc t) noexcept -> const GalGroupDelay* {
  update_tgd(ConstellationEnum::GAL, static_cast<GTime>(t));
  return this->gal_gd_.get();
}

auto EphemerisSolver::quary_glo_tgd(EpochUtc t) noexcept -> const GloGroupDelay* {
  update_tgd(ConstellationEnum::GLO, static_cast<GTime>(t));
  return this->glo_gd_.get();
}

auto EphemerisSolver::quary_qzs_tgd(EpochUtc t) noexcept -> const QzsGroupDelay* {
  update_tgd(ConstellationEnum::QZS, static_cast<GTime>(t));
  return this->qzs_gd_.get();
}

}  // namespace navp::sensors::gnss
#undef SQR