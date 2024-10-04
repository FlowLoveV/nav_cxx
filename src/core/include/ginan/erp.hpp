#pragma once

#include <cmath>
#include <map>
#include <vector>

#include "utils/gTime.hpp"

namespace navp::ginan {

// constexpr char eopComments[][16] = {"XP (MAS)", "YP (MAS)", "UT1(MTS)"};

/** earth rotation parameter data type
 */
struct ERPValues {
  utils::GTime time;

  f64 xp = 0;      ///< pole offset (rad)
  f64 yp = 0;      ///< pole offset (rad)
  f64 ut1Utc = 0;  ///< ut1-utc (s)
  f64 lod = 0;     ///< delta length of day (s/day)

  f64 xpr = 0;  ///< pole offset rate (rad/day)
  f64 ypr = 0;  ///< pole offset rate (rad/day)

  f64 xpSigma = 0;
  f64 ypSigma = 0;
  f64 xprSigma = 0;
  f64 yprSigma = 0;
  f64 ut1UtcSigma = 0;
  f64 lodSigma = 0;

  bool isPredicted = false;
  bool isFiltered = false;

  ERPValues operator+(const ERPValues& rhs) {
    ERPValues erpv = *this;

    erpv.time.bigTime += rhs.time.bigTime;
    erpv.xp += rhs.xp;
    erpv.yp += rhs.yp;
    erpv.ut1Utc += rhs.ut1Utc;
    erpv.lod += rhs.lod;

    erpv.xpr += rhs.xpr;
    erpv.ypr += rhs.ypr;

    erpv.xpSigma = sqrt((erpv.xpSigma * erpv.xpSigma) + (rhs.xpSigma * rhs.xpSigma));
    erpv.ypSigma = sqrt((erpv.ypSigma * erpv.ypSigma) + (rhs.ypSigma * rhs.ypSigma));
    erpv.xprSigma = sqrt((erpv.xprSigma * erpv.xprSigma) + (rhs.xprSigma * rhs.xprSigma));
    erpv.yprSigma = sqrt((erpv.yprSigma * erpv.yprSigma) + (rhs.yprSigma * rhs.yprSigma));
    erpv.ut1UtcSigma = sqrt((erpv.ut1UtcSigma * erpv.ut1UtcSigma) + (rhs.ut1UtcSigma * rhs.ut1UtcSigma));
    erpv.lodSigma = sqrt((erpv.lodSigma * erpv.lodSigma) + (rhs.lodSigma * rhs.lodSigma));

    erpv.isPredicted |= rhs.isPredicted;

    return erpv;
  }

  bool operator==(const ERPValues& rhs) const {
    bool equal = time == rhs.time && xp == rhs.xp && yp == rhs.yp && ut1Utc == rhs.ut1Utc && lod == rhs.lod &&
                 xpr == rhs.xpr && ypr == rhs.ypr;

    return equal;
  }

  ERPValues operator*(const f64 scalar) {
    ERPValues erpv = *this;

    erpv.time.bigTime *= scalar;
    erpv.xp *= scalar;
    erpv.yp *= scalar;
    erpv.ut1Utc *= scalar;
    erpv.lod *= scalar;

    erpv.xpr *= scalar;
    erpv.ypr *= scalar;

    erpv.xpSigma *= scalar;
    erpv.ypSigma *= scalar;
    erpv.xprSigma *= scalar;
    erpv.yprSigma *= scalar;
    erpv.ut1UtcSigma *= scalar;
    erpv.lodSigma *= scalar;

    return erpv;
  }

  std::string toString();
  std::string toReadableString();
};

struct ERP {
  std::vector<std::map<utils::GTime, ERPValues>> erpMaps;

  ERPValues filterValues;
};

}  // namespace navp::ginan