#include "sensors/gnss/atmosphere.hpp"

#include <numbers>

#define SQR(x) ((x) * (x))

/*
 * trop functions
 */
namespace navp::sensors::gnss::details {

#define ERR_TROP 3.0  ///< tropspheric delay std (m)
#define ERR_SAAS 0.3  ///< saastamoinen model error std (m)
#define ZEROC 273.15

struct TropSaasResult {
  f64 dry_map = 0;
  f64 wet_map = 0;
  f64 dry_ztd = 0;
  f64 wet_ztd = 0;
  f64 var = SQR(ERR_TROP);

  f64 trop() const noexcept { return dry_map * dry_ztd + wet_map * wet_ztd; }
};

constexpr f64 coefNMF[][5] = {{1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
                              {2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
                              {62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},

                              {0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
                              {0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
                              {0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},

                              {5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
                              {1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
                              {4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}};

// Herring model
f64 mapHerring(f64 el, f64 a, f64 b, f64 c) {
  f64 sinel = sin(el);
  return (1 + a / (1 + b / (1 + c))) / (sinel + (a / (sinel + b / (sinel + c))));
}

f64 interpc(const f64 coef[], f64 lat) {
  i32 i = (i32)(lat / 15);
  if (i < 1)
    return coef[0];
  else if (i > 4)
    return coef[4];
  return coef[i - 1] * (1 - lat / 15 + i) + coef[i] * (lat / 15 - i);
}

TropSaasResult tropSAAS(utils::GTime time, const utils::CoordinateBlh* pos, f64 el) {
  f64 lat = pos->x();
  f64 hgt = pos->z();
  if (hgt < -100 || hgt > +20000 || el < 0) {
    return TropSaasResult{};
  }
  TropSaasResult result;
  utils::UYds yds = time;
  /* year from doy 28, added half a year for southern latitudes */
  f64 y = (yds.doy - 28) / 365.25 + (lat < 0 ? 0.5 : 0);
  f64 cosy = cos(2 * std::numbers::pi * y);
  lat = fabs(lat);
  f64 ah[3];
  f64 aw[3];
  for (auto i = 0; i < 3; ++i) { /* year average           +    seasonal variation  */
    ah[i] = interpc(coefNMF[i], lat) - interpc(coefNMF[i + 3], lat) * cosy;
    aw[i] = interpc(coefNMF[i + 6], lat);
  }
  /* height correction */
  f64 dm = (1 / sin(el) - mapHerring(el, 2.53E-5, 5.49E-3, 1.14E-3)) * hgt / 1E3;
  result.dry_map = mapHerring(el, ah[0], ah[1], ah[2]) + dm;
  result.wet_map = mapHerring(el, aw[0], aw[1], aw[2]);
  f64 temp = 15 - 6.5E-3 * hgt + ZEROC;
  f64 pres = 1013.25 * pow(288.15 / temp, -5.255877);
  f64 e = 6.108 * 0.7 * exp((17.15 * temp - 4684) / (temp - 38.45));
  result.dry_ztd = 0.0022768 * pres / (1 - 0.00266 * cos(2 * pos->x()) - 0.00028 * hgt / 1E3);
  result.wet_ztd = 0.002277 * (1255 / temp + 0.05) * e;
  result.var = SQR(ERR_SAAS / (sin(el) + 0.1));
  return result;
}

}  // namespace navp::sensors::gnss::details

namespace navp::sensors::gnss {

AtmosphereHandler& AtmosphereHandler::set_time(const EpochUtc& tr) noexcept {
  tr_ = std::addressof(tr);
  return *this;
}

AtmosphereHandler& AtmosphereHandler::set_trop_model(TropModelEnum model) noexcept {
  trop_model_ = model;
  return *this;
}

AtmosphereHandler& AtmosphereHandler::set_iono_model(IonoModelEnum model) noexcept {
  iono_model_ = model;
  return *this;
}

AtmosphereHandler& AtmosphereHandler::set_sv_info(const EphemerisResult* eph_result) noexcept {
  sv_info_ = eph_result;
  return *this;
}

auto AtmosphereHandler::sv_info() const noexcept -> const EphemerisResult* { return sv_info_; }

bool AtmosphereHandler::solvable() const noexcept { return sv_info_; }

f64 AtmosphereHandler::handle_trop(const utils::CoordinateBlh* pos) const noexcept {
  if (!solvable()) return 0.0;
  switch (static_cast<TropModelEnum>(trop_model_)) {
    case TropModelEnum::STANDARD: {
      auto trop_saas_res = details::tropSAAS(static_cast<utils::GTime>(*tr_), pos, sv_info_->elevation);
      return trop_saas_res.trop();
    }
    case TropModelEnum::SBAS: {
      nav_error("not implmentted");
      return 0.0;
    }
    case TropModelEnum::VMF3: {
      nav_error("not implmentted");
      return 0.0;
    }
    case TropModelEnum::GPT2: {
      nav_error("not implmentted");
      return 0.0;
    }
    case TropModelEnum::CSSR: {
      nav_error("not implmentted");
      return 0.0;
    }
    default: {
      nav_error("Encountering an unexpected branch in TropModelEnum");
      return 0.0;
    }
  }
}

f64 AtmosphereHandler::handle_iono(const utils::CoordinateBlh* pos) const noexcept {
  if (!solvable()) return 0.0;
  switch (static_cast<IonoModelEnum>(iono_model_)) {
    case IonoModelEnum::NONE: {
      return 0.0;
    }
    case IonoModelEnum::MEAS_OUT: {
      nav_error("not implmented!");
      return 0.0;
    }
    case IonoModelEnum::BSPLINE: {
      nav_error("not implmented!");
      return 0.0;
    }
    case IonoModelEnum::SPHERICAL_CAPS: {
      nav_error("not implmented!");
      return 0.0;
    }
    case IonoModelEnum::SPHERICAL_HARMONICS: {
      nav_error("not implmented!");
      return 0.0;
    }
    case IonoModelEnum::LOCAL: {
      nav_error("not implmented!");
      return 0.0;
    }
    default: {
      nav_error("Encountering an unexpected branch in IonoModelEnum");
      return 0.0;
    }
  }
}

}  // namespace navp::sensors::gnss