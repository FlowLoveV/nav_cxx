#include "utils/gTime.hpp"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include <chrono>
#include <iostream>
#include <map>

#include "magic_enum.hpp"

using std::ostream;

#define S_IN_DAY 86400.0 /* Number of seconds in a day */
#define GPS_SUB_UTC_2000 +13
#define GPS_SUB_UTC_2006 +14
#define GPS_SUB_TAI -19

namespace navp::utils {

const GTime j2000TT =
    GEpoch{2000, (f64)MonthEnum::JAN, 1, 11, 58, 55.816 + GPS_SUB_UTC_2000};  // defined in utc 11:58:55.816
const GTime j2000Utc = GEpoch{2000, (f64)MonthEnum::JAN, 1, 12, 0, 0};        // right answer, wrong reason? todo

const GTime GPS_t0 = GEpoch{1980, (f64)MonthEnum::JAN, 6, 0, 0, 0};   // gps time reference
const GTime GLO_t0 = GEpoch{1980, (f64)MonthEnum::JAN, 6, 21, 0, 0};  // glo time reference (without leap seconds)
const GTime GAL_t0 = GEpoch{1999, (f64)MonthEnum::AUG, 22, 0, 0,
                            0};  // galileo system time reference as gps time -> 13 seconds before 0:00:00 UTC on
                                 // Sunday, 22 August 1999 (midnight between 21 and 22 August)
const GTime BDS_t0 =
    GEpoch{2006, (f64)MonthEnum::JAN, 1, 0,
           0,    0 + GPS_SUB_UTC_2006};  // beidou time reference as gps time - defined in utc 11:58:55.816

const i32 GPS_t0_sub_POSIX_t0 = 315964800;
const f64 MJD_j2000 = 51544.5;

const i32 secondsInWeek = 60 * 60 * 24 * 7;
const i32 secondsInDay = 60 * 60 * 24;
const f128 secondsInDayP = 60 * 60 * 24;

std::map<GTime, i32, std::greater<GTime>> leapSecondMap = {
    {GEpoch{2017, 1, 1, 0, 0, 18}, 18}, {GEpoch{2015, 7, 1, 0, 0, 17}, 17}, {GEpoch{2012, 7, 1, 0, 0, 16}, 16},
    {GEpoch{2009, 1, 1, 0, 0, 15}, 15}, {GEpoch{2006, 1, 1, 0, 0, 14}, 14}, {GEpoch{1999, 1, 1, 0, 0, 13}, 13},
    {GEpoch{1997, 7, 1, 0, 0, 12}, 12}, {GEpoch{1996, 1, 1, 0, 0, 11}, 11}, {GEpoch{1994, 7, 1, 0, 0, 10}, 10},
    {GEpoch{1993, 7, 1, 0, 0, 9}, 9},   {GEpoch{1992, 7, 1, 0, 0, 8}, 8},   {GEpoch{1991, 1, 1, 0, 0, 7}, 7},
    {GEpoch{1990, 1, 1, 0, 0, 6}, 6},   {GEpoch{1988, 1, 1, 0, 0, 5}, 5},   {GEpoch{1985, 7, 1, 0, 0, 4}, 4},
    {GEpoch{1983, 7, 1, 0, 0, 3}, 3},   {GEpoch{1982, 7, 1, 0, 0, 2}, 2},   {GEpoch{1981, 7, 1, 0, 0, 1}, 1},
    {GEpoch{1980, 1, 6, 0, 0, 0}, 0}};

ostream& operator<<(ostream& stream, const GTime& time) {
  stream << time.to_string();
  return stream;
}

ostream& operator<<(ostream& stream, const Duration& duration) {
  char buff[64];

  i32 decimal = (duration.bigTime - floor(duration.bigTime)) * 100;

  snprintf(buff, sizeof(buff), "%02d:%02d:%02d.%02d", (i32)duration.bigTime / 60 / 60, (i32)duration.bigTime / 60 % 60,
           (i32)duration.bigTime % 60, decimal);

  stream << buff;

  return stream;
}

/* convert substring in string to number
 * args   : char   *s        I   string ("... nnn.nnn ...")
 *          i32    i,n       I   substring position and width
 * return : converted number (0.0:error)
 */
f64 str2num(const char* s, i32 i, i32 n) {
  double value;
  char str[256], *p = str;

  if (i < 0 || (i32)strlen(s) < i || (i32)sizeof(str) - 1 < n) return 0.;

  for (s += i; *s && --n >= 0; s++) *p++ = *s == 'd' || *s == 'D' ? 'E' : *s;

  *p = '\0';
  return sscanf(str, "%lf", &value) == 1 ? static_cast<f64>(value) : 0;
}

/* convert substring in string to GTime struct
 * args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
 *          i32    i,n       I   substring position and width
 *          GTime *t       O   GTime struct
 * return : status (0:ok,0>:error)*/
i32 str2time(const char* s, i32 i, i32 n, GTime& t, TimeSystemEnum tsys) {
  f64 ep[6];
  char str[256], *p = str;

  if (i < 0 || (i32)strlen(s) < i || (i32)sizeof(str) - 1 < i) {
    return -1;
  }

  for (s += i; *s && --n >= 0;) {
    *p++ = *s++;
  }
  *p = '\0';

  i32 readCount = sscanf(str, "%lf %lf %lf %lf %lf %lf", ep, ep + 1, ep + 2, ep + 3, ep + 4, ep + 5);

  if (readCount < 6) {
    return -1;
  }

  if (ep[0] < 100) ep[0] += ep[0] < 80 ? 2000 : 1900;

  t = epoch2time(ep, tsys);

  return 0;
}

f64 leapSeconds(GTime time) {
  using namespace std::chrono;
  typedef std::chrono::duration<long, std::ratio<1, 1000000000>> duration_type;
  auto nanos = static_cast<long>(time.bigTime * 1e9);
  auto gtime = time_point<gps_clock, duration_type>{duration_type{nanos}};
  auto utc = clock_cast<utc_clock>(gtime);
  const auto _li = get_leap_second_info(utc);
  return static_cast<f64>(_li.elapsed.count()) - 9;
}

GTime yds2time(const f64* yds, TimeSystemEnum tsys) {
  i32 year = (i32)yds[0];
  i32 doy = (i32)yds[1];
  f64 sec = yds[2];

  if (year < 1970 || doy < 1 || doy > 366) {
    return GTime::noTime();
  }

  i32 leapDays =
      (year - 1968 - 1) / 4  // -1968 = last leap year before 1970; -1 = year must end before applying leap-day
      - (year - 1900 - 1) / 100 + (year - 1600 - 1) / 400;

  i32 days = (year - 1970) * 365 + leapDays + doy - 1;

  PTime pTime = {};
  pTime.bigTime = days * S_IN_DAY + sec;  //.0 to prevent eventual overflow

  GTime time = pTime;
  switch (tsys) {
    case TimeSystemEnum::GPST:
      break;  // nothing to do for now
    case TimeSystemEnum::GST:
      break;  // nothing to do for now
    case TimeSystemEnum::QZSST:
      break;  // nothing to do for now
    case TimeSystemEnum::BDT: {
      time += GPS_SUB_UTC_2006;
    } break;
    case TimeSystemEnum::GLONASST: {
      time -= 10800;
    }  // fallthough to account for leap seconds further
    case TimeSystemEnum::UTC: {
      UtcTime utcTime;
      utcTime.bigTime = time.bigTime;
      time = utcTime;
    } break;
    case TimeSystemEnum::TAI: {
      time += GPS_SUB_TAI;
    } break;
    default: {
    }
  }

  return time;
}

void time2yds(GTime time, f64* yds, TimeSystemEnum tsys) {
  GEpoch gEpoch;
  time2epoch(time, gEpoch.data(), tsys);

  // make new time with only the year of the input one,
  GEpoch gEpoch0(2000, 1, 1, 0, 0, 0);
  gEpoch0[0] = gEpoch[0];

  // subtract off the years
  Duration toy = (GTime)gEpoch - (GTime)gEpoch0;

  yds[0] = gEpoch0[0];
  yds[1] = toy.to_int() / 86400 + 1;  //(doy in bias SINEX (where yds is common) starts at 1)
  yds[2] = fmod(toy.to_double(), 86400);
}

UYds::operator GTime() const { return yds2time(this->data(), TimeSystemEnum::UTC); }

GTime epoch2time(const f64* ep, TimeSystemEnum tsys) {
  i32 year = (i32)ep[0];
  i32 mon = (i32)ep[1];
  i32 day = (i32)ep[2];
  i32 hour = (i32)ep[3];
  i32 min = (i32)ep[4];
  f64 sec = ep[5];

  if (year < 1970 || mon < 1 || mon > 12) {
    return GTime::noTime();
  }

  const i32 dayOffsetFromMonth[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  i32 dayOfYear = dayOffsetFromMonth[mon - 1] + day;

  if ((mon >= 3)          // after feb29
      && (year % 4 == 0)  // every 4 years
      && (year % 100 != 0 || year % 400 == 0)) {
    dayOfYear++;
  }

  f64 secOfDay = hour * 60 * 60 + min * 60 + sec;

  f64 yds[3];
  yds[0] = year;
  yds[1] = dayOfYear;
  yds[2] = secOfDay;
  GTime time = yds2time(yds, tsys);

  return time;
}

void time2epoch(GTime time, f64* ep, TimeSystemEnum tsys) {
  switch (tsys) {
    case TimeSystemEnum::GPST:
      break;  // nothing to do for now
    case TimeSystemEnum::GST:
      break;  // nothing to do for now
    case TimeSystemEnum::QZSST:
      break;  // nothing to do for now
    case TimeSystemEnum::BDT: {
      time -= GPS_SUB_UTC_2006;
    } break;
    case TimeSystemEnum::GLONASST: {
      time += 10800;
    }  // fallthough to account for leap seconds further
    case TimeSystemEnum::UTC: {
      UtcTime utcTime = time;
      time.bigTime = utcTime.bigTime;
    } break;
    case TimeSystemEnum::TAI: {
      time -= GPS_SUB_TAI;
    } break;
    default: {
    }
  }

  PTime pTime = time;

  const i32 mday[] = {/* # of days in a month */
                      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
                      31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  /* leap year if year % 4 == 0 in 1901-2099 */

  i32 days = (i32)(pTime.bigTime / secondsInDayP);
  f64 remSecs = pTime.bigTime - (time_t)days * secondsInDay;

  i32 doy = days % (365 * 4 + 1);
  i32 mon;
  for (mon = 0; mon < 48; mon++) {
    if (doy >= mday[mon])
      doy -= mday[mon];
    else
      break;
  }
  ep[0] = 1970 + days / 1461 * 4  // 1461 = 365.25 * 4
          + mon / 12;
  ep[1] = mon % 12 + 1;
  ep[2] = doy + 1;

  ep[3] = (i32)(remSecs / 3600);
  remSecs -= ep[3] * 3600;
  ep[4] = (i32)(remSecs / 60);
  remSecs -= ep[4] * 60;
  ep[5] = (remSecs);
}

/* convert week and tow in gps time to GTime struct
 * args   : i32    week      I   week number in gps time
 *          f64 sec       I   time of week in gps time (s)
 * return : GTime struct
 */
GTime gpst2time(i32 week, f64 sec) {
  GTime t = GPS_t0;

  if (sec < -1E9 || sec > 1E9) {
    sec = 0;
  }
  t.bigTime += secondsInWeek * week;
  t.bigTime += sec;

  return t;
}

UtcTime gpst2utc(GTime time) {
  f128 leaps = leapSeconds(time);

  UtcTime utcTime;
  utcTime.bigTime = time.bigTime - leaps;

  return utcTime;
}

GTime utc2gpst(UtcTime utcTime) {
  GTime gTime;
  gTime.bigTime = utcTime.bigTime;

  f64 leaps = leapSeconds(gTime);
  leaps = leapSeconds(gTime + leaps);

  gTime.bigTime += leaps;

  return gTime;
}

string GTime::to_string(i32 n) const {
  // if (cacheTime == bigTime && cacheN == n) {
  //   return cacheString;
  // }

  GTime t = *this;

  if (n < 0)
    n = 0;
  else if (n > 12)
    n = 12;

  f64 exper = pow(10, n);

  f128 val = t.bigTime * exper;
  val -= (i32)val;

  if (val > 0.5) {
    t.bigTime += 0.5 / exper;
  };

  GEpoch ep(t);

  char buff[64];
  snprintf(buff, sizeof(buff), "%04.0f-%02.0f-%02.0f %02.0f:%02.0f:%0*.*f", ep.year, ep.month, ep.day, ep.hour, ep.min,
           n <= 0 ? 2 : n + 3, n, ep.sec);

  // cacheString = buff;
  // cacheTime = bigTime;
  // cacheN = n;

  return std::string(buff);
}

string GTime::to_ISOstring(i32 n) const {
  string s = this->to_string(n);
  s[10] = 'T';
  return s;
}

f64 GTime::to_decYear() const {
  UYds yds = *this;

  f64 year = yds.year;
  f64 doy = yds.doy;
  f64 sod = yds.sod;

  // Determine if the year is a leap year
  bool isLeapYear =
      (static_cast<i32>(year) % 4 == 0 && static_cast<i32>(year) % 100 != 0) || (static_cast<i32>(year) % 400 == 0);
  i32 totalDaysInYear = isLeapYear ? 366 : 365;

  return year + (doy + sod / secondsInDay) / totalDaysInYear;
}

GTime::operator GEpoch() const {
  GEpoch gEpoch;
  time2epoch(*this, gEpoch.data());

  return gEpoch;
}

GTime GTime::floorTime(f64 period) const {
  GTime roundedTime = *this;

  // need separate functions for fractional / whole seconds
  // ignore fractions greater than one
  if (period < 1) {
    f64 fractionalSeconds = bigTime - (i32)bigTime;

    i32 wholePeriods = fractionalSeconds / period;

    fractionalSeconds = wholePeriods * period;

    roundedTime.bigTime = fractionalSeconds + (i32)bigTime;
  } else {
    // round to nearest chunk by integer arithmetic
    roundedTime.bigTime = ((i32)(roundedTime.bigTime / period)) * period;
  }

  return roundedTime;
}

/** Returns GTime in "dd-mmm-yyyy hh:mm:ss" format
 */
string GTime::gregString() {
  GEpoch epoch = *this;
  char buffer[25];
  snprintf(buffer, 25, "%02d-%3s-%04d %02d:%02d:%02d", (i32)epoch.day,
           std::string(magic_enum::enum_name<MonthEnum>(static_cast<MonthEnum>(epoch.month))).c_str(), (i32)epoch.year,
           (i32)epoch.hour, (i32)epoch.min, (i32)epoch.sec);
  return buffer;
}

/** Use a time of modulus and recent time to calculate the new time
 */
GTime nearestTime(GTime referenceEpoch,  //
                  f64 tom, GTime nearTime, i32 mod) {
  time_t seconds = (time_t)(nearTime.bigTime - referenceEpoch.bigTime);
  i32 nearMod = seconds / mod;
  i32 nearTom = seconds % mod;

  i32 deltaTom = tom - nearTom;

  i32 newMod = nearMod;

  if (deltaTom > +mod / 2) {
    newMod--;
  } else if (deltaTom < -mod / 2) {
    newMod++;
  }

  GTime newTime = referenceEpoch + newMod * mod + tom;

  return newTime;
}

GTime::operator MjDateTT() const {
  f128 thisDate = *this;
  f128 thenDate = j2000TT;

  f128 deltaDate = thisDate - thenDate;
  deltaDate /= secondsInDayP;

  MjDateTT mjd;
  mjd.val = MJD_j2000 + deltaDate;

  return mjd;
}

GTime::GTime(MjDateTT mjdTT) {
  f128 deltaDays = mjdTT.val - MJD_j2000;

  bigTime = j2000TT.bigTime + deltaDays * secondsInDayP;
}

GTime::GTime(MjDateUtc mjdUtc) {
  f128 deltaDays = mjdUtc.val - MJD_j2000;

  bigTime = j2000Utc.bigTime + deltaDays * secondsInDayP;

  f128 leaps = leapSeconds(*this);

  bigTime += leaps;
}

MjDateUt1::MjDateUt1(GTime time, f64 ut1_utc) {
  MjDateUtc mjdUtc = time;

  val = mjdUtc.val + ut1_utc / secondsInDayP;
}

MjDateUtc::MjDateUtc(GTime time) {
  f128 thisDate = time;
  f128 thenDate = j2000Utc;

  f128 deltaDate = thisDate - thenDate;
  deltaDate /= secondsInDayP;

  f128 leaps = leapSeconds(time);

  val = MJD_j2000 + deltaDate - leaps / secondsInDayP;
}

GTime::GTime(GWeek gpsWeek, GTow tow) { *this = GPS_t0 + gpsWeek * secondsInWeek + tow; }
GTime::GTime(BWeek bdsWeek, BTow tow) { *this = BDS_t0 + bdsWeek * secondsInWeek + tow; }

GEpoch ::operator GTime() const { return epoch2time(this->data()); }
UtcTime ::operator GTime() const { return utc2gpst(*this); }
GTime ::operator UtcTime() const { return gpst2utc(*this); }

GTime::GTime(GTow tow, GTime nearTime) { *this = nearestTime(GPS_t0, tow, nearTime, secondsInWeek); }
GTime::GTime(BTow tow, GTime nearTime) { *this = nearestTime(BDS_t0, tow, nearTime, secondsInWeek); }

GTime::GTime(RTod tod, GTime nearTime) {
  RTod nearTod = nearTime;

  f64 delta = tod - nearTod;

  while (delta > +secondsInDay / 2) delta -= secondsInDay;
  while (delta < -secondsInDay / 2) delta += secondsInDay;

  *this = nearTime + delta;
}

GTime::operator f128() const { return bigTime; }

GTime::operator GWeek() const {
  Duration seconds = *this - GPS_t0;
  GWeek gWeek = seconds.to_int() / secondsInWeek;
  return gWeek;
}
GTime::operator BWeek() const {
  Duration seconds = *this - BDS_t0;
  BWeek bWeek = seconds.to_int() / secondsInWeek;
  return bWeek;
}
GTime::operator GTow() const {
  Duration seconds = *this - GPS_t0;
  GTow gTow = fmod(seconds.to_double(), (f64)secondsInWeek);
  return gTow;
}
GTime::operator BTow() const {
  Duration seconds = *this - BDS_t0;
  BTow bTow = fmod(seconds.to_double(), (f64)secondsInWeek);
  return bTow;
}
GTime::operator RTod() const {
  Duration seconds = *this - GLO_t0;
  RTod rTod = fmod(seconds.to_double() - leapSeconds(*this), (f64)secondsInDay);
  return rTod;
}

PTime::operator GTime() const {
  GTime gTime;
  gTime.bigTime = bigTime - GPS_t0_sub_POSIX_t0;
  return gTime;
}
GTime::operator PTime() const {
  PTime pTime;
  pTime.bigTime = bigTime + GPS_t0_sub_POSIX_t0;
  return pTime;
}

GTime::operator string() const { return to_string(); }

}  // namespace navp::utils
