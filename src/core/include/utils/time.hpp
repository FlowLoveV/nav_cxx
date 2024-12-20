#pragma once

#include <chrono>
#include <compare>

#include "utils/exception.hpp"
#include "utils/hash.hpp"
#include "utils/logger.hpp"
#include "utils/macro.hpp"
#include "utils/result.hpp"
#include "utils/types.hpp"

namespace navp::details {
using namespace std;
using namespace std::chrono;

// clock begin at 2006/01/01 00:00:00
class bds_clock;

template <typename _Duration = std::chrono::duration<long, std::ratio<1, 1000000000>>>
using bds_time = time_point<bds_clock, _Duration>;
using bds_seconds = bds_time<seconds>;

class NAVP_EXPORT bds_clock {
 public:
  using rep = system_clock::rep;
  using period = system_clock::period;
  using duration = chrono::duration<rep, period>;
  using time_point = chrono::time_point<bds_clock>;
  static constexpr bool is_steady = false;  // XXX true for clock_typeAI?

  // TODO move into lib, use clock_typeAI on linux, add extension point.
  [[nodiscard]] static time_point now() { return from_utc(utc_clock::now()); }

  template <typename _Duration>
  [[nodiscard]] static utc_time<common_type_t<_Duration, seconds>> to_utc(const bds_time<_Duration>& __t) {
    using _CDur = common_type_t<_Duration, seconds>;
    return utc_time<_CDur>{__t.time_since_epoch()} + 1136073623s;
  }

  template <typename _Duration>
  [[nodiscard]] static bds_time<common_type_t<_Duration, seconds>> from_utc(const utc_time<_Duration>& __t) {
    using _CDur = common_type_t<_Duration, seconds>;
    return bds_time<_CDur>{__t.time_since_epoch()} - 1136073623s;
  }
};
}  // namespace navp::details

namespace std::chrono {
template <>
struct is_clock<navp::details::bds_clock> : true_type {};
template <>
inline constexpr bool is_clock_v<navp::details::bds_clock> = true;
}  // namespace std::chrono

template <typename _Duration, typename _CharT>
struct NAVP_EXPORT std::formatter<navp::details::bds_time<_Duration>, _CharT> : __format::__formatter_chrono<_CharT> {
  template <typename _ParseContext>
  constexpr typename _ParseContext::iterator parse(_ParseContext& __pc) {
    return _M_f._M_parse(__pc, __format::_ZonedDateTime);
  }

  template <typename _FormatContext>
  typename _FormatContext::iterator format(const navp::details::bds_time<_Duration>& __t, _FormatContext& __fc) const {
    constexpr chrono::days __bds_offset = chrono::days(3657 + 9492);
    using _CDur = common_type_t<_Duration, chrono::days>;
    chrono::local_time<_CDur> __lt(__t.time_since_epoch() + __bds_offset);
    const string __abbrev("BDS", 3);
    const chrono::seconds __off = 0s;
    const auto __lf = chrono::local_time_format(__lt, &__abbrev, &__off);
    return _M_f._M_format(__lf, __fc);
  }

 private:
  __format::__formatter_chrono<_CharT> _M_f;
};

namespace navp {

REGISTER_NAV_RUNTIME_ERROR_CHILD(TimeParseError, NavRuntimeError);

inline constexpr i64 millis_per_second = 1'000;
inline constexpr i64 micros_per_second = 1'000'000;
inline constexpr i64 nanos_per_second = 1'000'000'000;
inline constexpr i64 picos_per_second = 1'000'000'000'000;
inline constexpr i64 femtos_per_second = 1'000'000'000'000'000;
inline constexpr i64 attos_per_second = 1'000'000'000'000'000'000;
inline constexpr i64 seconds_per_minute = 60;
inline constexpr i64 seconds_per_hour = 3600;
inline constexpr i64 seconds_per_day = 86400;
inline constexpr i64 seconds_per_week = 604800;

template <typename T>
using Attoseconds = std::chrono::duration<T, std::ratio<1, attos_per_second>>;
template <typename T>
using Femtoseconds = std::chrono::duration<T, std::ratio<1, femtos_per_second>>;
template <typename T>
using Picoseconds = std::chrono::duration<T, std::ratio<1, picos_per_second>>;
template <typename T>
using Nanoseconds = std::chrono::duration<T, std::ratio<1, nanos_per_second>>;
template <typename T>
using Microseconds = std::chrono::duration<T, std::ratio<1, micros_per_second>>;
template <typename T>
using Milliseconds = std::chrono::duration<T, std::ratio<1, millis_per_second>>;
template <typename T>
using Seconds = std::chrono::duration<T>;
template <typename T>
using Minutes = std::chrono::duration<T, std::ratio<seconds_per_minute>>;
template <typename T>
using Hours = std::chrono::duration<T, std::ratio<seconds_per_hour>>;
template <typename T>
using Days = std::chrono::duration<T, std::ratio<seconds_per_day>>;
template <typename T>
using Weeks = std::chrono::duration<T, std::ratio<seconds_per_week>>;

/*  Singleton time zone information :
 *  - use `TimeZoneOffset::local_offset()` to get local time zone offest.
 *  - use `TimeZoneOffset::loacte_offset()` to get time zone offset by location name, may throw a std::runtime_error.
 *  - use `TimeZoneOffset::make_offset()` to make a time zone offset.
 *  - the time offset duration type is `std::chrono::duration<int32_t,std::ratio<1>>`.
 *  - Considering the widespread existence of daylight saving time and winter time in European and American countries,
 * the deviation of the same time zone within a year may change due to different seasons. Therefore, when obtaining the
 * time zone deviation, it is necessary to input time information. Examples :
 *  - int32_t offset_seconds = TimeZoneOffset::get_offset().count();
 */
struct NAVP_EXPORT TimeZoneOffset {
 public:
  typedef std::chrono::duration<i32, std::ratio<1>> duration_type;

  // get time zone offset :
  // - default get loacl time zone offset
  // - if the `set_offset()` function is not called, always return loacl time zone offset
  template <typename _Duration = std::chrono::nanoseconds>
  constexpr static auto local_offset(
      std::chrono::sys_time<_Duration> sys_time = std::chrono::system_clock::now()) noexcept -> const duration_type& {
    std::call_once(flag, [&]() {
      auto tz = std::chrono::current_zone();
      auto offset = tz->get_info(sys_time).offset;
      offset_seconds = std::chrono::duration_cast<duration_type>(offset);
    });
    return offset_seconds;
  }

  // get time zone offset by location name:
  // - see [examples](https://zh.cppreference.com/w/cpp/chrono/zoned_time)
  // - for linux/macos users,lookup /usr/share/zoneinfo to see the supported zone name
  // todo
  template <typename _Duration = std::chrono::nanoseconds>
  constexpr static duration_type locate_offset(
      const std::string_view& name, std::chrono::sys_time<_Duration> sys_time = std::chrono::system_clock::now()) {
    auto tz = std::chrono::locate_zone(name);
    auto offset = tz->get_info(sys_time).offset;
    return std::chrono::duration_cast<duration_type>(offset);
  };

  // make a time zone offset
  constexpr static duration_type make_offset(i32 seconds) noexcept { return duration_type(seconds); }

 protected:
  ~TimeZoneOffset() = default;

 private:
  TimeZoneOffset() = default;

  inline static std::mutex update_mutex{};
  inline static std::once_flag flag{};
  inline static duration_type offset_seconds{};
};

// Date with time zone offset, 24-hour clock system, features:
// - This Date type is only a date type and does not contain any information about the clock, which means you need to
// decide for yourself which clock the date is on
// - using `attoseconds` and `decimal_second` to improve the accuracy of expression in less than one second
// - when your operating platform `does not support f128`, please choose to use `attosecond`, which will provide you
// with an accuracy of `10^-18s`
// - when your operating platform `support f128`, you can get a better accuracy to use `decimal_second`, which accuracy
// usually can be `10^-33~10^-34s`
struct Date;

// week + seconds within a week + fractional second, features:
// - it does not specifically refer to the time of GPS or BDS systems, but only expresses a duration
// - using `attoseconds` and `decimal_second` to improve the accuracy of expression in less than one second
// - when your operating platform `does not support f128`, please choose to use `attosecond`, which will provide you
// with an accuracy of `10^-18s`
// - when your operating platform `support f128`, you can get a better accuracy to use `decimal_second`, which accuracy
// usually can be `10^-33~10^-34s`
struct GpsTime;

// time stamp generated by system,features:
// - unix time stamps, precision [ms]
// - using uint64_t to storage
struct TimeStamp;

template <typename clock_type>
class Epoch;

struct NAVP_EXPORT Date {
  i16 year;
  u16 month;
  u16 day;
  u16 hour;
  u16 minute;
  u16 integer_second;
  Seconds<i32> seconds_offset;  // default zero time zone
  // std::variant<i64, f_least64> decimal;  // either storage a i64 attoseconds or a float decimal second
  i64 attoseconds;           // i64 attoseconds(1e-18s)
  f_least64 decimal_second;  // float decimal second
};

struct NAVP_EXPORT GpsTime {
  u16 weeks;
  u32 integer_second;
  // std::variant<i64, f_least64> decimal;  // either storage a i64 attoseconds or a float decimal second
  i64 attoseconds;           // i64 attoseconds(1e-18s)
  f_least64 decimal_second;  // float decimal second
};

struct NAVP_EXPORT TimeStamp {
  u64 stamps;
};

namespace details {

using std::chrono::year_month_day;

constexpr year_month_day tai_begin_ymd = 1958y / 1 / 1;
constexpr year_month_day utc_begin_ymd = 1970y / 1 / 1;
constexpr year_month_day sys_begin_ymd = 1970y / 1 / 1;
constexpr year_month_day gps_begin_ymd = 1980y / 1 / 6;
constexpr year_month_day bds_begin_ymd = 2006y / 1 / 1;

constexpr i64 bds_to_utc_offset = 1136073623;
constexpr i64 gps_to_utc_offset = 315964809;
constexpr i64 tai_to_utc_offset = -378691210;

// prcise 1e-18s
struct Duration {
  typedef Seconds<i64> integer_duration_type;
  typedef Attoseconds<i64> decimal_duration_type;
  typedef std::variant<i64, f_least64> decimal_type;

  // i64 `-9,223,372,036,854,775,808` to `9,223,372,036,854,775,807`
  Seconds<i64> _m_seconds{};
  Attoseconds<i64> _m_attos{};

  // return to duration of clock_type_in
  template <typename in_clock_t, typename out_clock_t>
  constexpr static Duration from_date(u16 year, u8 month, u16 day, u8 hour, u8 minute, u8 integer_second,
                                      decimal_type decimal_second, Seconds<i32> offset_seconds) {
    if (!(check_ymd(year, month, day) && check_hms(hour, minute, integer_second, decimal_second))) {
      throw "Incorrect use of date time constructor";
    }
    auto duration = duration_from_date<in_clock_t>(year, month, day, hour, minute, integer_second);
    duration -= integer_duration_type(offset_seconds);
    std::chrono::time_point<in_clock_t, integer_duration_type> input_time(duration);
    auto output_time = std::chrono::clock_cast<out_clock_t>(input_time);
    auto attos = std::holds_alternative<i64>(decimal_second)
                     ? std::get<i64>(decimal_second)
                     : static_cast<i64>(std::get<f_least64>(decimal_second) * attos_per_second);
    return Duration{._m_seconds = output_time.time_since_epoch(), ._m_attos = decimal_duration_type{attos}};
  }

  template <typename in_clock_t, typename out_clock_t, typename second_t>
  constexpr static auto from_gps_time(u32 weeks, second_t seconds) -> Duration {
    auto dur = Duration::from_seconds<in_clock_t, out_clock_t, second_t>(seconds);
    dur.add_weeks(weeks);
    return dur;
  }

  template <typename in_clock_t, typename out_clock_t, typename second_t>
  constexpr static auto from_seconds(second_t seconds) -> Duration {
    i64 sec = 0, atto = 0;
    if constexpr (std::is_integral_v<second_t>) {
      sec = static_cast<i64>(seconds);
    } else {
      f128 integral;
      f128 fractional = std::modf(static_cast<double>(seconds), &integral);
      sec = static_cast<i64>(integral);
      atto = static_cast<i64>(fractional * 1e18);
    }
    integer_duration_type integer_dur(sec);
    auto in_tp = std::chrono::time_point<in_clock_t, integer_duration_type>(integer_dur);
    auto this_tp = std::chrono::clock_cast<out_clock_t>(in_tp);
    decimal_duration_type decimal_dur(atto);
    // // correct leap seconds
    // leap_second_info leap;
    // if constexpr (std::is_same_v<in_clock_t, utc_clock>) {
    //   auto utc_tp = chrono::utc_time<integer_duration_type>(integer_dur);
    //   leap = get_leap_second_info(utc_tp);
    //   utc_tp += leap.elapsed;
    //   leap = get_leap_second_info(utc_tp);
    //   integer_dur += leap.elapsed;
    // }
    return Duration{._m_seconds = this_tp.time_since_epoch(), ._m_attos = decimal_dur};
  };

  // // return duration independent of clock type
  // constexpr static Duration from_gps_time(const u16 weeks, u32 integer_second, decimal_type decimal_second) {
  //   f64 sec = static_cast<f64>(integer_second);
  //   if (!check_gps_time(weeks, sec)) {
  //     throw "Incorrect use of gps time constructor";
  //   }
  //   auto integer_duration = std::chrono::weeks(weeks) + std::chrono::seconds(integer_second);
  //   auto attos = std::holds_alternative<i64>(decimal_second)
  //                    ? std::get<i64>(decimal_second)
  //                    : static_cast<i64>(std::get<f_least64>(decimal_second) * attos_per_second);
  //   return Duration{._m_seconds = duration_cast<integer_duration_type>(integer_duration),
  //                   ._m_attos = decimal_duration_type{attos}};
  // }

  // constexpr static Duration from_seconds(f_least64 seconds) noexcept {
  //   assert(seconds < std::numeric_limits<i64>::max() && seconds > std::numeric_limits<i64>::min());
  //   auto integer = static_cast<i64>(seconds);
  //   auto fractional = seconds - (f_least64)(integer);
  //   return Duration{._m_seconds = integer_duration_type(integer),
  //                   ._m_attos = decimal_duration_type((i64)(fractional * attos_per_second))};
  // }

  inline constexpr i64 weeks() const noexcept { return _m_seconds.count() / seconds_per_week; }
  inline constexpr i64 days() const noexcept { return _m_seconds.count() / seconds_per_day; }
  inline constexpr i64 hours() const noexcept { return _m_seconds.count() / seconds_per_hour; }
  inline constexpr i64 minutes() const noexcept { return _m_seconds.count() / seconds_per_minute; }
  inline constexpr i64 seconds() const noexcept { return _m_seconds.count(); }
  inline constexpr i64 fractional_milliseconds() const noexcept {
    return _m_attos.count() / (attos_per_second / millis_per_second);
  }
  inline constexpr i64 fractional_microseconds() const noexcept {
    return _m_attos.count() / (attos_per_second / micros_per_second);
  }
  inline constexpr i64 fractional_nanoseconds() const noexcept {
    return _m_attos.count() / (attos_per_second / nanos_per_second);
  }
  inline constexpr i64 fractional_picoseconds() const noexcept {
    return _m_attos.count() / (attos_per_second / picos_per_second);
  }
  inline constexpr i64 fractional_femtoseconds() const noexcept {
    return _m_attos.count() / (attos_per_second / femtos_per_second);
  }
  inline constexpr i64 fractional_attoseconds() const noexcept { return _m_attos.count(); }
  inline constexpr f_least64 scale_fractional_seconds(u8 factor = 0) const noexcept {
    assert(factor <= 18);
    return (f_least64)(_m_attos.count() * pow(10, factor)) / attos_per_second;
  }

  constexpr void add_weeks(i64 week) noexcept { _m_seconds += integer_duration_type(week * seconds_per_week); }
  constexpr void add_days(i64 day) noexcept { _m_seconds += integer_duration_type(day * seconds_per_day); }
  constexpr void add_hours(i64 hour) noexcept { _m_seconds += integer_duration_type(hour * seconds_per_hour); }
  constexpr void add_minutes(i64 minute) noexcept { _m_seconds += integer_duration_type(minute * seconds_per_minute); }
  constexpr void add_seconds(i64 second) noexcept { _m_seconds += integer_duration_type(second); }
  constexpr void add_milliseconds(i64 millisecond) noexcept {
    auto attos = millisecond * (attos_per_second / millis_per_second) + _m_attos.count();
    adjust_attos(*this);
  }
  constexpr void add_microseconds(i64 microsecond) noexcept {
    auto attos = microsecond * (attos_per_second / micros_per_second) + _m_attos.count();
    adjust_attos(*this);
  }
  constexpr void add_nanoseconds(i64 nanosecond) noexcept {
    auto attos = nanosecond * (attos_per_second / nanos_per_second) + _m_attos.count();
    adjust_attos(*this);
  }
  constexpr void add_picoseconds(i64 picosecond) noexcept {
    auto attos = picosecond * (attos_per_second / picos_per_second) + _m_attos.count();
    adjust_attos(*this);
  }
  constexpr void add_femtoseconds(i64 femtosecond) noexcept {
    auto attos = femtosecond * (attos_per_second / femtos_per_second) + _m_attos.count();
    adjust_attos(*this);
  }
  constexpr void add_attoseconds(i64 attosecond) noexcept {
    auto attos = attosecond + _m_attos.count();
    adjust_attos(*this);
  }
  constexpr void add_fractional_second(f_least64 frantional_second) noexcept {
    assert(frantional_second < (f_least64)1.0);
    auto attos = (i64)(frantional_second * attos_per_second) + _m_attos.count();
    adjust_attos(*this);
  }

  // date
  template <typename clock_type_in, typename clock_type_out>
  constexpr Date date(Seconds<i32> seconds_offset) noexcept {
    integer_duration_type integer_seconds = _m_seconds + seconds_offset;
    // to system clock duration
    auto in_tp = std::chrono::time_point<clock_type_in, integer_duration_type>(integer_seconds);
    auto out_tp = std::chrono::clock_cast<clock_type_out>(in_tp);
    auto sys_tp = std::chrono::clock_cast<system_clock>(out_tp);
    auto day = std::chrono::floor<Days<i64>>(sys_tp.time_since_epoch());
    integer_duration_type frac = sys_tp.time_since_epoch() - day;
    auto ymd = std::chrono::year_month_day(std::chrono::sys_time<Days<i64>>(day));
    auto hms = std::chrono::hh_mm_ss<integer_duration_type>(frac);
    return Date{.year = static_cast<i16>(i32(ymd.year())),
                .month = static_cast<u16>(u32(ymd.month())),
                .day = static_cast<u16>(u32(ymd.day())),
                .hour = static_cast<u16>(hms.hours().count()),
                .minute = static_cast<u16>(hms.minutes().count()),
                .integer_second = static_cast<u16>(hms.seconds().count()),
                .seconds_offset = seconds_offset,
                .attoseconds = _m_attos.count(),
                .decimal_second = static_cast<f_least64>(_m_attos.count() / 1e18)};
  }

  // gps time
  constexpr GpsTime gps_time() noexcept {
    auto week = weeks();
    auto second = _m_seconds.count() - week * seconds_per_week;
    return GpsTime{.weeks = static_cast<u16>(week),
                   .integer_second = static_cast<u32>(second),
                   .attoseconds = _m_attos.count(),
                   .decimal_second = static_cast<f_least64>(_m_attos.count() / 1e18)};
  }

  constexpr auto operator<=>(const Duration& rhs) const noexcept {
    if (auto cmp = _m_seconds <=> rhs._m_seconds; cmp != 0) {
      return cmp;
    }
    return _m_attos <=> rhs._m_attos;
  }

  constexpr Duration operator+(const Duration& rhs) const noexcept {
    auto add_dur = Duration{
        ._m_seconds = _m_seconds + rhs._m_seconds,
        ._m_attos = _m_attos + rhs._m_attos,
    };
    adjust_attos(add_dur);
    return add_dur;
  }

  template <typename _Dur>
  constexpr Duration operator+(const _Dur& _dur) const noexcept {
    static_assert(std::is_same_v<_Dur, std::chrono::duration<typename _Dur::rep, typename _Dur::period>>,
                  "operator+: _Dur must be a std::chrono::duration type");

    using namespace std::chrono;
    auto total_seconds = duration_cast<Seconds<i64>>(_dur);
    auto fractional_seconds = _dur - duration_cast<_Dur>(total_seconds);
    auto total_attoseconds = duration_cast<Attoseconds<i64>>(fractional_seconds);

    auto result = Duration{._m_seconds = _m_seconds + total_seconds, ._m_attos = _m_attos + total_attoseconds};

    adjust_attos(result);

    return result;
  };

  constexpr Duration operator-(const Duration& rhs) const noexcept {
    auto minus_dur = Duration{
        ._m_seconds = _m_seconds - rhs._m_seconds,
        ._m_attos = _m_attos - rhs._m_attos,
    };
    adjust_attos(minus_dur);
    return minus_dur;
  }

  template <typename _Dur>
  constexpr Duration operator-(const _Dur& _dur) const noexcept {
    static_assert(std::is_same_v<_Dur, std::chrono::duration<typename _Dur::rep, typename _Dur::period>>,
                  "operator+: _Dur must be a std::chrono::duration type");

    using namespace std::chrono;
    auto total_seconds = duration_cast<Seconds<i64>>(_dur);
    auto fractional_seconds = _dur - duration_cast<_Dur>(total_seconds);
    auto total_attoseconds = duration_cast<Attoseconds<i64>>(fractional_seconds);

    auto result = Duration{._m_seconds = _m_seconds - total_seconds, ._m_attos = _m_attos - total_attoseconds};

    adjust_attos(result);

    return result;
  };

 protected:
  static constexpr void adjust_attos(Duration& _dur) noexcept {
    if (_dur._m_attos.count() >= attos_per_second) {
      auto extra_seconds = _dur._m_attos.count() / attos_per_second;
      _dur._m_seconds += integer_duration_type(extra_seconds);
      _dur._m_attos = decimal_duration_type(_dur._m_attos.count() % attos_per_second);
    } else if (_dur._m_attos.count() < 0) {
      auto deficit_seconds = -_dur._m_attos.count() / attos_per_second;
      _dur._m_seconds -= integer_duration_type(deficit_seconds + 1);
      _dur._m_attos = decimal_duration_type(_dur._m_attos.count() % attos_per_second + attos_per_second);
    }
  }

  template <typename date_clock_t>
  constexpr static auto duration_from_date(u16 year, u8 month, u16 day, u8 hour, u8 minute,
                                           u8 integer_second) -> integer_duration_type {
    auto y = std::chrono::year(year);
    auto m = std::chrono::month(month);
    auto d = std::chrono::day(day);
    auto ymd = std::chrono::year_month_day(y, m, d);
    year_month_day ymd0;
    if constexpr (std::is_same_v<date_clock_t, utc_clock>) {
      ymd0 = utc_begin_ymd;
    } else if constexpr (std::is_same_v<date_clock_t, gps_clock>) {
      ymd0 = gps_begin_ymd;
    } else if constexpr (std::is_same_v<date_clock_t, bds_clock>) {
      ymd0 = bds_begin_ymd;
    } else if constexpr (std::is_same_v<date_clock_t, tai_clock>) {
      ymd0 = tai_begin_ymd;
    } else if constexpr (std::is_same_v<date_clock_t, system_clock>) {
      ymd0 = sys_begin_ymd;
    }
    const auto days = std::chrono::sys_days{ymd};
    const auto days0 = std::chrono::sys_days{ymd0};
    auto _duration =
        days - days0 + std::chrono::hours(hour) + std::chrono::minutes(minute) + std::chrono::seconds(integer_second);
    auto duration = duration_cast<integer_duration_type>(_duration);
    // correct leap seconds
    leap_second_info leap;
    if constexpr (std::is_same_v<date_clock_t, utc_clock>) {
      auto utc_tp = chrono::utc_time<integer_duration_type>(duration);
      leap = get_leap_second_info(utc_tp);
      utc_tp += leap.elapsed;
      leap = get_leap_second_info(utc_tp);
      duration += leap.elapsed;
    }
    return duration;
  }

  constexpr static bool check_ymd(u16 year, u8 month, u16 day) {
    std::chrono::year_month_day ymd{chrono::year(year), chrono::month(month), chrono::day(day)};
    if (ymd.ok()) {
      return true;
    } else {
      nav_error("Invalid year/month/day : The Date {}-{}-{} is not vaild.", year, month, day);
      return false;
    }
  }

  constexpr static bool check_hms(u8 hour, u8 minute, u8 integer_second, decimal_type decimal_second) {
    if (std::holds_alternative<i64>(decimal_second)) {
      auto val = std::get<i64>(decimal_second);
      if (val >= attos_per_second) {
        nav_error("Invalid decimal_seconds : {}attos , beyond 1s", val);
      }
    } else {
      auto val = std::get<f_least64>(decimal_second);
      if (val >= 1) {
        nav_error("Invalid decimal_seconds : {}s , beyond 1s", static_cast<f64>(val));
      }
    }
    if (hour >= 24 || minute >= 60 || integer_second >= 60) {
      nav_error("Invalid hh:mm:ss : The {}:{}:{} is not vaild.", hour, minute, integer_second);
      return false;
    }
    return true;
  }

  constexpr static bool check_gps_time(const u16 weeks, const f64 seconds) {
    if (seconds >= 604800) {
      nav_error("The range of seconds in weeks should be in [0,604800),but get {}.", seconds);
      return false;
    }
    if ((weeks * 604800 + seconds) * 1e9 >= static_cast<f64>(numeric_limits<long>::max())) {
      nav_error(
          "The range of GPS exceeds the maximum value that can be represented by the data "
          "structure,but get {} {} .",
          weeks, seconds);
      return false;
    }
    return true;
  }
};

static constexpr navp::i64 _format_parse_ratio_val =
    std::is_same_v<navp::f_least64, navp::f64> ? navp::nanos_per_second : navp::femtos_per_second / 10;
using _format_parse_rep = std::conditional_t<std::is_same_v<navp::f_least64, navp::f64>, navp::i64, navp::f_least64>;
using _format_parse_ratio = std::ratio<1, _format_parse_ratio_val>;
using _format_parse_cdur = std::chrono::duration<_format_parse_rep, _format_parse_ratio>;

template <typename clock_type>
_format_parse_cdur get_dur(const navp::Epoch<clock_type>& epoch) noexcept {
  return chrono::duration_cast<_format_parse_cdur>(epoch.__dur._m_seconds) +
         chrono::duration_cast<_format_parse_cdur>(epoch.__dur._m_attos);
}

}  // namespace details

template <typename dst_clock_type, typename src_clock_type>
NAVP_EXPORT Epoch<dst_clock_type> clock_cast(const Epoch<src_clock_type>& src) {
  if constexpr (std::is_same_v<dst_clock_type, src_clock_type>) {
    return src;
  } else {
    std::chrono::time_point<src_clock_type, typename Epoch<src_clock_type>::integer_duration_type> tp_src(
        src.__dur._m_seconds);
    std::chrono::time_point<dst_clock_type, typename Epoch<src_clock_type>::integer_duration_type> tp_dst =
        std::chrono::clock_cast<dst_clock_type>(tp_src);
    return Epoch<dst_clock_type>{.__dur = details::Duration{
                                     ._m_seconds = tp_dst.time_since_epoch(),
                                     ._m_attos = src.__dur._m_attos,
                                 }};
  }
}

// utc_clock / gps_clock / bds_clock / system_clock:
// - clock precise is nanos,means you can't get any higher precise time from these clock
template <typename clock_type>
class NAVP_EXPORT Epoch {
  static_assert(std::chrono::is_clock_v<clock_type>, "Epoch should use a stdardand clock type!");
  static_assert(!std::is_same_v<clock_type, std::chrono::local_t>,
                "Epoch does't support local_t,use TimeZoneOffset instead!");

 public:
  typedef details::Duration::integer_duration_type integer_duration_type;
  typedef details::Duration::decimal_duration_type decimal_duration_type;

  // todo
  // now only works for utc_clock
  constexpr static Result<Epoch, TimeParseError> from_str(const char* fmt, const char* ctx) {
    using duration_type = std::chrono::duration<details::_format_parse_rep, details::_format_parse_ratio>;
    std::chrono::time_point<clock_type, duration_type> base_tp;
    details::istringstream ss(ctx);
    ss >> std::chrono::parse(fmt, base_tp);
    if (ss.fail()) {
      return Err(TimeParseError(std::format("Can't parse \'{}\' into Epoch with format \'{}\'", ctx, fmt)));
    }
    // adjust
    if constexpr (!std::is_same_v<clock_type, std::chrono::utc_clock>) {
      std::chrono::utc_time<duration_type> utc_tp(base_tp.time_since_epoch());
      base_tp = clock_cast<clock_type>(utc_tp);
    }
    auto _dur = base_tp.time_since_epoch();
    auto _seconds = std::chrono::duration_cast<integer_duration_type>(_dur);
    auto _frac = std::chrono::duration_cast<decimal_duration_type>(_dur - _seconds);
    return Ok(Epoch{.__dur = details::Duration{._m_seconds = _seconds, ._m_attos = _frac}});
  }

  static Epoch now(TimeZoneOffset::duration_type offset = TimeZoneOffset::duration_type()) {
    auto _tp = clock_type::now();
    auto _nanos = _tp.time_since_epoch().count();
    auto _integer_seconds = _nanos / nanos_per_second;
    _integer_seconds += offset.count();
    auto _nanoseconds = _nanos % nanos_per_second;
    return Epoch{.__dur = {details::Duration{._m_seconds = integer_duration_type(_integer_seconds),
                                             ._m_attos = decimal_duration_type(_nanoseconds * 1'000'000'000)}}};
  }

  template <typename clock_type_in = clock_type>
  constexpr static Epoch from_date(const Date& date) {
    // `details::Duration::from_date` returns to
    return Epoch{.__dur = details::Duration::from_date<clock_type_in, clock_type>(
                     date.year, date.month, date.day, date.hour, date.minute, date.integer_second,
                     judge_input(date.attoseconds, date.decimal_second), date.seconds_offset)};
  }

  template <typename in_clock_t = clock_type>
  constexpr static Epoch from_gps_time(const GpsTime& gps_time) {
    auto dur = details::Duration::from_gps_time<in_clock_t, clock_type>(gps_time.weeks, gps_time.integer_second);
    auto decimal_second = judge_input(gps_time.attoseconds, gps_time.decimal_second);
    auto attos = std::holds_alternative<i64>(decimal_second)
                     ? std::get<i64>(decimal_second)
                     : static_cast<i64>(std::get<f_least64>(decimal_second) * attos_per_second);
    return Epoch{.__dur = details::Duration{
                     ._m_seconds = dur._m_seconds,
                     ._m_attos = decimal_duration_type(attos),
                 }};
  }

  template <typename in_clock_t = clock_type, typename second_t>
  constexpr static Epoch from_gps_time(u32 weeks, second_t seconds) {
    return Epoch{.__dur = details::Duration::from_gps_time<in_clock_t, clock_type, second_t>(weeks, seconds)};
  }

  template <typename in_clock_t = clock_type>
  constexpr static Epoch from_seconds(f_least64 seconds) {
    return Epoch{.__dur = details::Duration::from_seconds<in_clock_t>(seconds)};
  }

  // default get zero time zone date
  template <typename clock_type_out = clock_type>
  constexpr Date date(Seconds<i32> time_zone_offset = Seconds<i32>()) {
    return __dur.date<clock_type, clock_type_out>(time_zone_offset);
  }

  constexpr GpsTime gps_time() { return __dur.gps_time(); }

  constexpr i64 weeks() const noexcept { return __dur.weeks(); }
  constexpr i64 days() const noexcept { return __dur.days(); }
  constexpr i64 hours() const noexcept { return __dur.hours(); }
  constexpr i64 minutes() const noexcept { return __dur.minutes(); }
  constexpr i64 seconds() const noexcept { return __dur.seconds(); }
  constexpr i64 fractional_milliseconds() const noexcept { return __dur.fractional_milliseconds(); }
  constexpr i64 fractional_microseconds() const noexcept { return __dur.fractional_microseconds(); }
  constexpr i64 fractional_nanoseconds() const noexcept { return __dur.fractional_nanoseconds(); }
  constexpr i64 fractional_picoseconds() const noexcept { return __dur.fractional_picoseconds(); }
  constexpr i64 fractional_femtoseconds() const noexcept { return __dur.fractional_femtoseconds(); }
  constexpr i64 fractional_attoseconds() const noexcept { return __dur.fractional_attoseconds(); }
  constexpr f_least64 scale_fractional_seconds(u8 factor = 0) const noexcept {
    return __dur.scale_fractional_seconds(factor);
  }

  constexpr void add_weeks(i64 week) noexcept { __dur.add_weeks(week); }
  constexpr void add_days(i64 day) noexcept { __dur.add_days(day); }
  constexpr void add_hours(i64 hour) noexcept { __dur.add_hours(hour); }
  constexpr void add_minutes(i64 minute) noexcept { __dur.add_minutes(minute); }
  constexpr void add_seconds(i64 second) noexcept { __dur.add_seconds(second); }
  constexpr void add_milliseconds(i64 millisecond) noexcept { __dur.add_milliseconds(millisecond); }
  constexpr void add_microseconds(i64 microsecond) noexcept { __dur.add_microseconds(microsecond); }
  constexpr void add_nanoseconds(i64 nanosecond) noexcept { __dur.add_nanoseconds(nanosecond); }
  constexpr void add_picoseconds(i64 picosecond) noexcept { __dur.add_picoseconds(picosecond); }
  constexpr void add_femtoseconds(i64 femtosecond) noexcept { __dur.add_femtoseconds(femtosecond); }
  constexpr void add_attoseconds(i64 attosecond) noexcept { __dur.add_attoseconds(attosecond); }
  constexpr void add_fractional_second(f_least64 fractional_second) noexcept {
    __dur.add_fractional_second(fractional_second);
  }

  template <typename U = clock_type>
  constexpr auto operator<=>(const Epoch<U>& rhs) const noexcept {
    Epoch _cast_rhs = clock_cast<clock_type>(rhs);
    return __dur <=> _cast_rhs.__dur;
  }

  template <typename U = clock_type>
  constexpr Epoch operator+(const Epoch<U>& rhs) const noexcept {
    Epoch _cast_rhs = clock_cast<clock_type>(rhs);
    return Epoch{.__dur = __dur + _cast_rhs.__dur};
  }

  template <typename U = clock_type>
  constexpr Epoch operator-(const Epoch<U>& rhs) const noexcept {
    Epoch _cast_rhs = clock_cast<clock_type>(rhs);
    return Epoch{.__dur = __dur - _cast_rhs.__dur};
  }

  template <typename _Dur>
  constexpr Epoch operator+(const _Dur& _dur) const noexcept {
    auto result_dur = this->__dur.template operator+ <_Dur>(_dur);
    return Epoch{.__dur = result_dur};
  }

  template <typename _Dur>
  constexpr Epoch operator-(const _Dur& _dur) const noexcept {
    auto result_dur = this->__dur.template operator- <_Dur>(_dur);
    return Epoch{.__dur = result_dur};
  }

  details::Duration __dur{};

 protected:
  typedef details::Duration::decimal_type decimal_type;

  constexpr static decimal_type judge_input(i64 attos, f_least64 decimal_sec) noexcept {
    bool using_i64 = attos != 0, using_f = decimal_sec != 0;
    if (!using_f && !using_i64) return 0;
    // else if (using_f) [[likely]]
    // In most scenarios, people are accustomed to using floating-point numbers to
    // represent parts of less than one second
    else if (using_f)
      return decimal_sec;
    else
      return attos;
  }
};

typedef Epoch<std::chrono::system_clock> EpochSys;
typedef Epoch<std::chrono::utc_clock> EpochUtc;
typedef Epoch<std::chrono::tai_clock> EpochTai;
typedef Epoch<std::chrono::gps_clock> EpochGps;
typedef Epoch<navp::details::bds_clock> EpochBds;

// some custom hash for Epoch
// template <typename clock_type>
// struct NAV_EXPORT EpochHasherSecond()

}  // namespace navp

namespace std {
// std::hash
template <typename clock_type>
struct NAV_EXPORT hash<navp::Epoch<clock_type>> {
  size_t operator()(const navp::Epoch<clock_type>& epoch) const noexcept {
    std::size_t seed = 0;
    navp::hash_combine(seed, epoch.__dur._m_seconds.count());
    navp::hash_combine(seed, epoch.__dur._m_attos.count());
    return seed;
  }
};

// std::formatter
template <typename clock_type, typename _CharT>
struct NAVP_EXPORT formatter<navp::Epoch<clock_type>, _CharT> : __format::__formatter_chrono<_CharT> {
  template <typename _ParseContext>
  constexpr typename _ParseContext::iterator parse(_ParseContext& __pc) {
    return _M_f._M_parse(__pc, __format::_ZonedDateTime);
  }

  template <typename _FormatContext>
  typename _FormatContext::iterator format(const navp::Epoch<clock_type>& epoch, _FormatContext& __fc) const {
    // check if exceed range
    if (epoch.seconds() > std::numeric_limits<navp::i64>::max() / navp::nanos_per_second) {
      nav_error(
          "The given time exceeds the maximum time that can be represented, the data will overflow and be formatted "
          "incorrectly");
    }

    /*
     * gps time formatter settings
     */
    typedef navp::Days<_Rep> _Days;
    using std::chrono::duration_cast;
    // Convert to __local_time_fmt with abbrev "GPS" and offset 0s.
    // Offset is 1980y/January/Sunday[1] - 1970y/January/1
    constexpr auto __gps_offset = _Days(3657);
    const string __abbrev_gps("GPS", 3);
    // Convert to __local_time_fmt with abbrev "BDS" and offset 0s.
    // Offset is 2006y/January/1 - 1970y/January/1
    constexpr auto __bds_offset = _Days(13149);
    const string __abbrev_bds("BDS", 3);
    // Convert to __local_time_fmt with abbrev "TAI" and offset 0s.
    // Offset is 1970y/January/1 - 1958y/January/1
    constexpr auto __tai_offset = _Days(-4383);
    const string __abbrev_tai("TAI", 3);
    const chrono::seconds __off = 0s;  // all offset for gps time is 0s

    auto format_utc = [&](const _CDur& _dur) {
      using chrono::__detail::__utc_leap_second;
      chrono::utc_time<_CDur> __u(_dur);
      const auto __li = chrono::get_leap_second_info(__u);
      chrono::sys_time<_CDur> __s{_dur - __li.elapsed};
      if (!__li.is_leap_second) [[likely]]
        return _M_f._M_format(__s, __fc);
      else
        return _M_f._M_format(__utc_leap_second(__s), __fc);
    };

    if constexpr (std::is_same_v<clock_type, std::chrono::utc_clock>) {
      // format utc_clock
      _CDur _dur = navp::details::get_dur(epoch);
      return format_utc(_dur);
    } else if constexpr (std::is_same_v<clock_type, std::chrono::gps_clock>) {
      // format gps_clock
      _CDur _dur = navp::details::get_dur(epoch);
      chrono::local_time<_CDur> __lt(_dur + duration_cast<_CDur>(__gps_offset));
      const auto __lf = chrono::local_time_format(__lt, &__abbrev_gps, &__off);
      return _M_f._M_format(__lf, __fc);
    } else if constexpr (std::is_same_v<clock_type, navp::details::bds_clock>) {
      // format bds_clock
      _CDur _dur = navp::details::get_dur(epoch);
      chrono::local_time<_CDur> __lt(_dur + duration_cast<_CDur>(__bds_offset));
      const auto __lf = chrono::local_time_format(__lt, &__abbrev_bds, &__off);
      return _M_f._M_format(__lf, __fc);
    } else if constexpr (std::is_same_v<clock_type, std::chrono::system_clock>) {
      // format system_clock
      _CDur _dur = navp::details::get_dur(epoch);
      chrono::sys_time<_CDur> _t(_dur);
      return _M_f._M_format(_t, __fc);
    } else if constexpr (std::is_same_v<clock_type, std::chrono::tai_clock>) {
      // format tai_clock
      _CDur _dur = navp::details::get_dur(epoch);
      chrono::local_time<_CDur> __lt(_dur + duration_cast<_CDur>(__tai_offset));
      const auto __lf = chrono::local_time_format(__lt, &__abbrev_tai, &__off);
      return _M_f._M_format(__lf, __fc);
    } else {
      return std::format_to(__fc, "");
    }
  }

 private:
  using _Rep = navp::details::_format_parse_rep;
  using _CDur = navp::details::_format_parse_cdur;
  __format::__formatter_chrono<_CharT> _M_f;
};
}  // namespace std
