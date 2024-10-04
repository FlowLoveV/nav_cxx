
#pragma once

#include <array>
#include <iostream>
#include <string>

#include "utils/types.hpp"
#include "sensors/gnss/enums.hpp"

namespace navp::utils {

using sensors::gnss::MonthEnum;
using sensors::gnss::TimeSystemEnum;
using std::array;
using std::ostream;
using std::string;

struct PTime;
struct GTime;
struct UtcTime;
struct GEpoch;
struct GWeek;
struct GTow;
struct MjDateTT;
struct MjDateUtc;

#define GPS_SUB_UTC_2000 +13
#define GPS_SUB_UTC_2006 +14
#define GPS_SUB_TAI -19

extern const GTime GPS_t0;
extern const f64 MJD_j2000;
extern const i32 secondsInDay;

string time2str(GTime t, i32 n);

GTime yds2time(const f64* yds, TimeSystemEnum tsys = TimeSystemEnum::GPST);

void time2yds(GTime time, f64* yds, TimeSystemEnum tsys = TimeSystemEnum::GPST);

GTime epoch2time(const f64* ep, TimeSystemEnum tsys = TimeSystemEnum::GPST);

void time2epoch(GTime time, f64* ep, TimeSystemEnum tsys = TimeSystemEnum::GPST);

f64 leapSeconds(GTime time);

struct Int {
  i32 val = 0;

  Int() {}

  Int(const i32& in) : val{in} {}

  operator i32() const { return val; }

  Int& operator=(const i32& in) {
    val = in;
    return *this;
  }
};

struct Double {
  f64 val = 0;

  Double() {}

  Double(const f64& in) : val{in} {}

  operator f64() const { return val; }

  Double& operator=(const f64& in) {
    val = in;
    return *this;
  }
};

struct GWeek : Int {
  GWeek(i32 in) { val = in; }
};

struct BWeek : Int {
  BWeek(i32 in) { val = in; }
};

struct GTow : Double {
  GTow(f64 in) { val = in; }
};

struct BTow : Double {
  BTow(f64 in) { val = in; }
};

struct RTod : Double {
  RTod(f64 in) { val = in; }
};

struct Duration {
  f128 bigTime = 0;

  f64 to_double() const { return (f64)bigTime; }

  i32 to_int() const { return (i32)bigTime; }

  bool operator<(const f64& t2) const {
    if (this->bigTime < t2)
      return true;
    else
      return false;
  }

  bool operator>(const f64& t2) const {
    if (this->bigTime > t2)
      return true;
    else
      return false;
  }

  f128 operator-(const Duration& t2) const { return this->bigTime - t2.bigTime; }

  f64 operator/(const Duration& t2) const { return static_cast<f64>((this->bigTime / t2.bigTime)); }

  friend ostream& operator<<(ostream& os, const Duration& time);
};

/** Time structure used throughout this software
 */
struct GTime {
  mutable i32 cacheN = 0;
  mutable f128 cacheTime = -1;
  mutable string cacheString;

  f128 bigTime = 0;

  /** Uninitialised time for comparisons
   */
  static GTime noTime() {
    GTime nothing;
    return nothing;
  }

  GTime(GTow tow, GTime nearTime);

  GTime(BTow tow, GTime nearTime);

  GTime(RTod tod, GTime nearTime);

  string to_string(i32 n = 2) const;

  string to_ISOstring(i32 n = 2) const;

  f64 to_decYear() const;

  bool operator==(const GTime& t2) const {
    if (this->bigTime != t2.bigTime)
      return false;
    else
      return true;
  }

  bool operator!=(const GTime& t2) const { return !(*this == t2); }

  bool operator<(const GTime& t2) const {
    if (this->bigTime < t2.bigTime)
      return true;
    else
      return false;
  }

  bool operator>(const GTime& t2) const {
    if (this->bigTime > t2.bigTime)
      return true;
    else
      return false;
  }

  bool operator>=(const GTime& t2) const {
    if (*this > t2) return true;
    if (*this == t2)
      return true;
    else
      return false;
  }

  friend ostream& operator<<(ostream& os, const GTime& time);

  GTime operator+(const f64 t) const {
    GTime gTime = *this;

    gTime.bigTime += t;

    return gTime;
  }

  GTime operator+(const i32 t) const {
    GTime gTime = *this;

    gTime.bigTime += t;

    return gTime;
  }

  GTime operator+(const Duration duration) const {
    GTime gTime = *this;

    gTime.bigTime += duration.bigTime;

    return gTime;
  }

  GTime& operator+=(const f64 rhs) {
    *this = *this + rhs;
    return *this;
  }

  GTime& operator-=(const f64 rhs) {
    *this = *this - rhs;
    return *this;
  }

  Duration operator-(const GTime t) const {
    Duration duration;
    duration.bigTime = bigTime - t.bigTime;

    return duration;
  }

  GTime operator-(const f64 t) const {
    GTime time = *this + (-t);
    return time;
  }

  GTime operator-(const Duration duration) const {
    GTime gTime = *this;
    gTime.bigTime -= duration.bigTime;
    return gTime;
  }

  GTime& operator++(i32) {
    this->bigTime++;
    return *this;
  }

  GTime() {}

  GTime(GWeek gpsWeek, GTow tow);

  GTime(BWeek bdsWeek, BTow tow);

  GTime(MjDateTT mjdTT);

  GTime(MjDateUtc mjdUtc);

  GTime floorTime(f64 period) const;

  string gregString();

  operator f128() const;
  operator MjDateTT() const;
  operator GEpoch() const;
  operator UtcTime() const;
  operator GWeek() const;
  operator BWeek() const;
  operator GTow() const;
  operator BTow() const;
  operator PTime() const;
  operator string() const;
  operator RTod() const;
};

struct PTime {
  f128 bigTime = 0;

  PTime() {}

  operator GTime() const;
};

GTime timeGet();

struct MjDateUtc {
  f128 val;

  MjDateUtc() {}

  MjDateUtc(GTime time);

  f64 to_double() const { return (f64)val; }
};

struct MjDateUt1 {
  f128 val;

  MjDateUt1() {}

  MjDateUt1(GTime time, f64 ut1_utc);

  f64 to_double() const { return (f64)val; }

  f64 to_j2000() const { return (f64)(val - MJD_j2000); }
};

struct MjDateTT {
  f128 val;

  f64 to_double() const { return (f64)val; }

  f64 to_j2000() const { return (f64)(val - MJD_j2000); }
};

struct UtcTime {
  f128 bigTime;  // Eugene: bigTime can be ambiguous, e.g. 1167264000.5, never know if GPST is 2017-01-01
                        // 00:00:17.5 or 2017-01-01 00:00:18.5

  UtcTime operator+(const f64 t) const {
    UtcTime time = *this;

    time.bigTime += t;

    return time;
  }

  string to_string(i32 n = 2) const {
    GTime gTime;
    gTime.bigTime = this->bigTime;
    string str = gTime.to_string(n);
    str += "Z";
    str[10] = 'T';
    return str;
  }

  string to_ISOstring(i32 n = 2) const {
    GTime gTime;
    gTime.bigTime = this->bigTime;

    return gTime.to_ISOstring(n) + 'Z';
  }

  UtcTime() {}

  operator GTime() const;
};

struct GEpoch : array<f64, 6> {
  GTime toGTime() const;

  operator GTime() const;

  f64& year;
  f64& month;
  f64& day;
  f64& hour;
  f64& min;
  f64& sec;

  GEpoch(f64 yearVal = 0, f64 monthVal = 0, f64 dayVal = 0, f64 hourVal = 0, f64 minVal = 0,
         f64 secVal = 0)
      : year{(*this)[0]}, month{(*this)[1]}, day{(*this)[2]}, hour{(*this)[3]}, min{(*this)[4]}, sec{(*this)[5]} {
    year = yearVal;
    month = monthVal;
    day = dayVal;
    hour = hourVal;
    min = minVal;
    sec = secVal;
  }

  GEpoch(const GEpoch& other)
      : year{(*this)[0]}, month{(*this)[1]}, day{(*this)[2]}, hour{(*this)[3]}, min{(*this)[4]}, sec{(*this)[5]} {
    // special copy constructor to deal with aliases
    year = other.year;
    month = other.month;
    day = other.day;
    hour = other.hour;
    min = other.min;
    sec = other.sec;
  }

  GEpoch& operator=(const GEpoch& other) {
    year = other.year;
    month = other.month;
    day = other.day;
    hour = other.hour;
    min = other.min;
    sec = other.sec;

    return *this;
  }
};

struct UYds : array<f64, 3> {
  f64& year;
  f64& doy;
  f64& sod;

  UYds(f64 yearval = 0, f64 doyVal = 0, f64 sodVal = 0) : year{(*this)[0]}, doy{(*this)[1]}, sod{(*this)[2]} {
    year = yearval;
    doy = doyVal;
    sod = sodVal;
  }

  UYds(const UYds& yds) : year{(*this)[0]}, doy{(*this)[1]}, sod{(*this)[2]} {
    // special copy constructor to deal with aliases
    year = yds.year;
    doy = yds.doy;
    sod = yds.sod;
  }

  UYds& operator=(const UYds& other) {
    year = other.year;
    doy = other.doy;
    sod = other.sod;

    return *this;
  }

  UYds(const GTime& time) : year{(*this)[0]}, doy{(*this)[1]}, sod{(*this)[2]} {
    time2yds(time, this->data(), TimeSystemEnum::UTC);
  }

  UYds& operator+=(const f64 offset) {
    sod += offset;
    while (sod > secondsInDay) {
      sod -= secondsInDay;
      doy++;
    }
    while (sod < 0) {
      sod += secondsInDay;
      doy--;
    }

    while (doy > 366) {
      doy -= 365;
      year++;
    }
    while (doy < 1) {
      doy += 365;
      year--;
    }

    return *this;
  }

  UYds& operator=(const GTime& time) {
    *this = UYds(time);

    return *this;
  }

  operator GTime() const;
};

UtcTime gpst2utc(GTime t);
GTime utc2gpst(UtcTime t);

f64 str2num(const char* s, i32 i, i32 n);

GTime gpst2time(i32 week, f64 sec);
f64 time2gpst(GTime t, i32* week = nullptr);

GTime bdt2time(i32 week, f64 sec);
f64 time2bdt(GTime t, i32* week = nullptr);

i32 str2time(const char* s, i32 i, i32 n, GTime& t, TimeSystemEnum tsys = TimeSystemEnum::GPST);

void jd2ymdhms(const f64 jd, f64* ep);

f64 ymdhms2jd(const f64 time[6]);

GTime nearestTime(GTime referenceEpoch, f64 tom, GTime nearTime, i32 mod);
}  // namespace navp::utils
