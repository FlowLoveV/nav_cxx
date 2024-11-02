#pragma once

#include <rfl.hpp>

#include "logger.hpp"
#include "types.hpp"
#include "utils/error.hpp"
#include "utils/gTime.hpp"
#include "utils/macro.hpp"
#include "utils/result.hpp"

// [formatter-doc](https://zh.cppreference.com/w/cpp/chrono/system_clock/formatter)
// [parse-doc](https://zh.cppreference.com/w/cpp/chrono/parse)
// [doc](https://zh.cppreference.com/w/cpp/chrono#.E6.97.A5.E5.8E.86)

namespace navp::details {
using namespace std;
using namespace std::chrono;

// 2006/01/01 00:00:00
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
  static constexpr bool is_steady = false;  // XXX true for CLOCK_TAI?

  // TODO move into lib, use CLOCK_TAI on linux, add extension point.
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

enum TimeScaleEnum : u8 {
  UTC,
  GPST,
  BDT,
};

template <TimeScaleEnum T>
struct Epoch;

// epoch_cast
template <TimeScaleEnum Dst, TimeScaleEnum Src>
NAVP_EXPORT Epoch<Dst> epoch_cast(const Epoch<Src>& src) {
  if constexpr (Dst == Src) {
    return src;
  } else {
    return std::chrono::clock_cast<typename Epoch<Dst>::clock_type>(src.tp);
  }
}

namespace details {

// reflect epoch helper class
struct EpochReflectHelper {
  rfl::Field<"time", rfl::Timestamp<"%Y-%m-%d %H:%M:%S">> time;
};

// convert a f64 second to [long second,long nanosecond]
NAVP_EXPORT std::tuple<long, long> convert_seconds(f64 seconds);

// convert [u8 second,long nanosecond] to a f64 second
NAVP_EXPORT f64 convert_seconds(long second, long nanos);

NAVP_EXPORT bool is_leap_year(i32 year);

template <typename T>
struct EpochPayload {
  // reflect helper
  using ReflectionType = EpochReflectHelper;
  ReflectionType reflection() const {
    const auto sys_time_point = std::chrono::clock_cast<std::chrono::system_clock>(this->time_point());
    const std::time_t t = std::chrono::system_clock::to_time_t(sys_time_point);
    const auto tm = std::gmtime(&t);
    return ReflectionType{.time = *tm};
  }

  // to f128 seconds
  operator f128() const noexcept { return static_cast<f128>(this->time_point().time_since_epoch().count()); }
  // to long nanos
  operator long() const noexcept { return this->time_point().time_since_epoch().count(); }
  // plus / minus opearator
  template <class Dur>
  constexpr T& operator+=(const Dur& dur) {
    auto p = static_cast<T*>(this);
    p->tp += dur;
    return *p;
  }
  template <class Dur>
  constexpr T& operator-=(const Dur& dur) {
    auto p = static_cast<T*>(this);
    p->tp -= dur;
    return *p;
  }
  template <typename Dur>
  constexpr T operator+(const Dur& dur) {
    return *static_cast<T*>(this) + dur;
  }
  template <typename Dur>
  constexpr T operator-(const Dur& dur) {
    return *static_cast<T*>(this) - dur;
  }

  // auto-increment,auto-decrement opearator
  constexpr T& operator++() { static_cast<T*>(this)++; }
  constexpr T& operator--() { static_cast<T*>(this)--; }
  constexpr T& operator++(i32 _dur) { static_cast<T*>(this)++(_dur); }
  constexpr T& operator--(i32 _dur) { static_cast<T*>(this)--(_dur); }

  // now
  constexpr static T now() {
    typedef typename T::clock_type clock_t;
    return clock_t::now();
  }
  // max min
  constexpr static auto max() -> T {
    typedef typename T::time_point_type time_point_t;
    return T{time_point_t::max()};
  }
  constexpr static auto min() -> T {
    typedef typename T::time_point_type time_point_t;
    return T{time_point_t::min()};
  }

  constexpr auto time_point() const { return static_cast<const T*>(this)->tp; }
  constexpr auto time_point() { return static_cast<T*>(this)->tp; }

  // from string
  // set from utc time string
  constexpr void set_from_str(const char* fmt, const char* ctx) {
    istringstream ss(ctx);
    ss >> std::chrono::parse(fmt, static_cast<T*>(this)->tp);
    if (ss.fail()) {
      nav_error("Failed to parse datetime string : Can't format string \"{}\" with fmt \"{}\".", ctx, fmt);
      cpptrace::generate_trace(1).print_with_snippets();
    }
  }
  // construct from utc time string
  constexpr static NavResult<T> from_str(const char* fmt, const char* ctx) {
    std::chrono::time_point<utc_clock, std::chrono::duration<long, std::ratio<1, 1000000000>>> base_tp;
    istringstream ss(ctx);
    ss >> std::chrono::parse(fmt, base_tp);
    if (ss.fail()) {
      return Err(errors::NavError::Utils::Time::ParseEpochError);
      // nav_error("Failed to parse datetime string : Can't format string \"{}\" with fmt \"{}\".", ctx, fmt);
    }
    return Ok(T{clock_cast<typename T::clock_type>(base_tp)});
  }

  // from nanos
  constexpr static T from_nanos(long nanos) noexcept {
    return T{typename T::time_point_type{typename T::duration_type{nanos}}};
  }

  // from seconds
  constexpr static T from_seconds(f128 seconds) noexcept {
    auto f = seconds * 1e9;
    long nanos = static_cast<long>(f);
    if ((f - nanos) != 0.0) {
      nav_warn("There is a loss of precision when converting f128 seconds to long nanoseconds");
    }
    return EpochPayload::from_nanos(nanos);
  }

  // from gps time
  template <typename U = T>
  constexpr static enable_if_t<
      std::disjunction_v<std::is_same<U, Epoch<navp::GPST>>, std::is_same<U, Epoch<navp::BDT>>>, U>
  from_gps_time(const u16 weeks, u32 seconds, long nanos) {
    auto sec = convert_seconds(seconds, nanos);
    if (!check_gps_time(weeks, sec)) {
      throw "Incorrect use of gps time constructor";
    }
    auto duration = std::chrono::weeks(weeks) + std::chrono::seconds(seconds) + std::chrono::nanoseconds(nanos);
    return T{duration};
  }
  template <typename U = T>
  constexpr static enable_if_t<
      std::disjunction_v<std::is_same<U, Epoch<navp::GPST>>, std::is_same<U, Epoch<navp::BDT>>>, U>
  from_gps_time(const u16 weeks, f64 seconds) {
    if (!check_gps_time(weeks, seconds)) {
      throw "Incorrect use of gps time constructor";
    }
    auto [sec, nano] = convert_seconds(seconds);
    return EpochPayload::from_gps_time(weeks, sec, nano);
  }
  // from date

  // from local date
  template <typename U = T>
  constexpr static enable_if_t<std::disjunction_v<std::is_same<U, Epoch<navp::UTC>>>, U> from_local_date(
      const u16 year, const u8 month, const u16 day, const u8 hour, const u8 minute, const u8 seconds,
      const long nanos) {
    if (!(check_ymd(year, month, day) && check_ymd_range(year, month, day) &&
          check_hms(hour, minute, seconds, nanos))) {
      throw "Incorrect use of date time constructor";
    }
    auto y = std::chrono::year(year);
    auto m = std::chrono::month(month);
    auto d = std::chrono::day(day);
    auto ymd = std::chrono::year_month_day(y, m, d);
    const auto days = std::chrono::sys_days{ymd}.time_since_epoch();
    auto duration = days + std::chrono::hours(hour) + std::chrono::minutes(minute) + std::chrono::seconds(seconds) +
                    std::chrono::nanoseconds(nanos);
    std::chrono::time_point<std::chrono::local_t, typename T::duration_type> local_time(duration);
    // to target clock
    auto sys_time = std::chrono::current_zone()->to_sys(local_time);
    auto time = std::chrono::clock_cast<typename T::clock_type>(sys_time);
    return U{time};
  }

  // from local date
  template <typename U = T>
  constexpr static enable_if_t<std::disjunction_v<std::is_same<U, Epoch<navp::UTC>>>, U> from_local_date(
      const u16 year, const u8 month, const u16 day, const u8 hour, const u8 minute, const f64 seconds) {
    auto [secs, nanos] = convert_seconds(seconds);
    return EpochPayload::from_local_date(year, month, day, hour, minute, secs, nanos);
  }

  template <typename U = T>
  constexpr static enable_if_t<std::disjunction_v<std::is_same<U, Epoch<navp::UTC>>>, U> from_utc_date(
      const u16 year, const u8 month, const u16 day, const u8 hour, const u8 minute, const u8 seconds,
      const long nanos) {
    if (!(check_ymd(year, month, day) && check_ymd_range(year, month, day) &&
          check_hms(hour, minute, seconds, nanos))) {
      throw "Incorrect use of date time constructor";
    }
    auto y = std::chrono::year(year);
    auto m = std::chrono::month(month);
    auto d = std::chrono::day(day);
    auto ymd = std::chrono::year_month_day(y, m, d);
    const auto days = std::chrono::sys_days{ymd}.time_since_epoch();
    auto duration = days + std::chrono::hours(hour) + std::chrono::minutes(minute) + std::chrono::seconds(seconds) +
                    std::chrono::nanoseconds(nanos);
    std::chrono::time_point<system_clock> system_time(duration);
    // to target clock
    auto time = std::chrono::clock_cast<typename T::clock_type>(system_time);
    return U{time};
  }

  // from local date
  template <typename U = T>
  constexpr static enable_if_t<std::disjunction_v<std::is_same<U, Epoch<navp::UTC>>>, U> from_utc_date(
      const u16 year, const u8 month, const u16 day, const u8 hour, const u8 minute, const f64 seconds) {
    auto [secs, nanos] = convert_seconds(seconds);
    return EpochPayload::from_utc_date(year, month, day, hour, minute, secs, nanos);
  }
  // from
  // get gps time information
  template <typename U = T>
  constexpr enable_if_t<std::disjunction_v<std::is_same<U, Epoch<navp::GPST>>, std::is_same<U, Epoch<navp::BDT>>>,
                        std::tuple<u16, f64>>
  gps_time() {
    auto nanoseconds = this->time_point().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(nanoseconds).count();
    u16 weeks = seconds / 604800;
    f64 tow = (seconds % 604800) + (nanoseconds.count() % 1000000000) / 1e9;
    return std::make_tuple(weeks, tow);
  }
  // get date information
  constexpr auto days_since_epoch() { return this->time_point_of_days_since_epoch().time_since_epoch().count(); }
  constexpr auto nanoseconds_within_day() { return this->duration_of_nanoseconds_within_day().count(); }
  constexpr auto seconds_within_day() { return this->duration_of_seconds_within_day().count(); }

  constexpr static auto ymd_of(const T& t, bool is_local = false) -> std::tuple<i32, unsigned, unsigned> {
    auto tp = t.tp;
    // Considering that the starting points of different time scales are inconsistent, overflow may
    // occur during conversion, so it is necessary to unify the starting points of the time scales
    // first.
    if constexpr (std::same_as<T, Epoch<GPST>>) {
      tp -= chrono::days(3657);
    } else if constexpr (std::same_as<T, Epoch<BDT>>) {
      tp -= chrono::days(3657 + 9492);
    }
    auto sys_tp = std::chrono::clock_cast<std::chrono::system_clock>(tp);
    if constexpr (std::same_as<T, Epoch<GPST>>) {
      sys_tp += chrono::days(3657);
    } else if constexpr (std::same_as<T, Epoch<BDT>>) {
      sys_tp += chrono::days(3657 + 9492);
    }
    std::chrono::year_month_day ymd;
    if (is_local) {
      auto local_tp = std::chrono::current_zone()->to_local(sys_tp);
      ymd = std::chrono::year_month_day{std::chrono::floor<std::chrono::days>(local_tp)};
    } else {
      ymd = std::chrono::year_month_day{std::chrono::floor<std::chrono::days>(sys_tp)};
    }
    return std::make_tuple(i32(ymd.year()), unsigned(ymd.month()), unsigned(ymd.day()));
  }

  constexpr static auto hms_of(const T& t, bool is_local = false) -> std::tuple<long, long, long, long> {
    const auto sys_tp = std::chrono::clock_cast<system_clock>(t.tp);
    if (is_local) {
      auto local_tp = std::chrono::current_zone()->to_local(sys_tp);
      auto hms = std::chrono::hh_mm_ss{local_tp.time_since_epoch() % std::chrono::days(1)};
      return std::make_tuple(hms.hours().count(), hms.minutes().count(), hms.seconds().count(),
                             hms.subseconds().count());
    }
    auto hms = std::chrono::hh_mm_ss{sys_tp.time_since_epoch() % std::chrono::days(1)};
    return std::make_tuple(hms.hours().count(), hms.minutes().count(), hms.seconds().count(), hms.subseconds().count());
  }

  constexpr static auto date_of(const T& t,
                                bool is_local = false) -> std::tuple<i32, unsigned, unsigned, long, long, long, long> {
    auto [y, m, d] = EpochPayload::ymd_of(t, is_local);
    auto [H, M, S, N] = EpochPayload::hms_of(t, is_local);
    return std::make_tuple(y, m, d, H, M, S, N);
  }

  constexpr auto local_date() -> std::tuple<i32, unsigned, unsigned, long, long, long, long> {
    return EpochPayload::date_of(*static_cast<T*>(this), true);
  }
  constexpr auto utc_date() -> std::tuple<i32, unsigned, unsigned, long, long, long, long> {
    return EpochPayload::date_of(*static_cast<T*>(this), false);
  }
  // helper
 protected:
  constexpr auto time_point_of_days_since_epoch() {
    return std::chrono::floor<std::chrono::days>(static_cast<T*>(this)->tp);
  }
  constexpr auto duration_of_nanoseconds_within_day() {
    return static_cast<T*>(this)->tp - this->time_point_of_days_since_epoch();
  }
  constexpr auto duration_of_seconds_within_day() {
    return duration_cast<std::chrono::duration<f64>>(this->nanoseconds_within_day());
  }
  // check date
  constexpr static bool check_ymd(const u16 year, const u8 month, const u16 day) {
    std::chrono::year_month_day ymd{chrono::year(year), chrono::month(month), chrono::day(day)};
    if (ymd.ok()) {
      return true;
    } else {
      nav_error("Invalid year/month/day : The Date {}-{}-{} is not vaild.", year, month, day);
      return false;
    }
  }
  constexpr static bool check_ymd_range(const u16 year, const u8 month, const u16 day) {
    if constexpr (std::same_as<T, Epoch<UTC>>) {
      constexpr auto max_ymd = chrono::year_month_day(chrono::year(2262), chrono::month(4), chrono::day(10));
      constexpr auto min_ymd = chrono::year_month_day(chrono::year(1677), chrono::month(9), chrono::day(21));
      std::chrono::year_month_day ymd{chrono::year(year), chrono::month(month), chrono::day(day)};
      if (min_ymd <= ymd && max_ymd >= ymd) {
        return true;
      } else {
        nav_error("Year/month/day out of range : The Date {}-{}-{} exceeds range.", year, month, day);
        return false;
      }
    } else if constexpr (std::same_as<T, Epoch<GPST>>) {
      constexpr auto max_ymd = chrono::year_month_day(chrono::year(2272), chrono::month(4), chrono::day(14));
      constexpr auto min_ymd = chrono::year_month_day(chrono::year(1687), chrono::month(9), chrono::day(27));
      std::chrono::year_month_day ymd{chrono::year(year), chrono::month(month), chrono::day(day)};
      if (min_ymd <= ymd && max_ymd >= ymd) {
        return true;
      } else {
        nav_error("Year/month/day out of range : The Date {}-{}-{} exceeds range.", year, month, day);
        return false;
      }
    } else if constexpr (std::same_as<T, Epoch<BDT>>) {
      constexpr auto max_ymd = chrono::year_month_day(chrono::year(2272), chrono::month(4), chrono::day(14));
      constexpr auto min_ymd = chrono::year_month_day(chrono::year(1687), chrono::month(9), chrono::day(27));
      std::chrono::year_month_day ymd{chrono::year(year), chrono::month(month), chrono::day(day)};
      if (min_ymd <= ymd && max_ymd >= ymd) {
        return true;
      } else {
        nav_error("Year/month/day out of range : The Date {}-{}-{} exceeds range.", year, month, day);
        return false;
      }
    }
    return true;
  }
  constexpr static bool check_hms(const u8 hour, const u8 minute, const u8 seconds, const long nanos) {
    if (hour >= 24 || minute >= 60 || seconds >= 60 || nanos < 0 || nanos >= 1000000000) {
      nav_error("Invalid hh:mm:ss : The {}:{}:{}:{} is not vaild.", hour, minute, seconds, nanos);
      return false;
    }
    return true;
  }
  // check gps time
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

};  // namespace details

template <>
struct NAVP_EXPORT Epoch<UTC> : public details::EpochPayload<Epoch<UTC>> {
  typedef std::chrono::utc_clock clock_type;
  typedef std::chrono::duration<long, std::ratio<1, 1000000000>> duration_type;
  typedef std::chrono::time_point<clock_type, duration_type> time_point_type;

  time_point_type tp;
  using details::EpochPayload<Epoch<UTC>>::EpochPayload;
  // default constructor
  constexpr Epoch() = default;
  constexpr Epoch(long t) noexcept : tp(duration_type(t)) {}
  constexpr Epoch(const time_point_type& _tp) : tp(_tp) {}
  constexpr Epoch(time_point_type&& _tp) : tp(std::move(_tp)) {}
  constexpr Epoch& operator=(const time_point_type& _tp) {
    if (&this->tp == &_tp) {
      return *this;
    } else {
      this->tp = _tp;
      return *this;
    }
  }
  constexpr Epoch& operator=(time_point_type&& _tp) {
    if (&this->tp == &_tp) {
      return *this;
    } else {
      this->tp = std::move(_tp);
      return *this;
    }
  }
  ~Epoch() = default;
  constexpr explicit Epoch(const duration_type& dur) : tp(dur) {}

  // Interacting with the ginan library's gTime type
  Epoch(utils::GTime gtime) noexcept;
  operator utils::GTime() const noexcept;

  // reflect constructor
  Epoch(const ReflectionType& _impl);
};

template <>
struct NAVP_EXPORT Epoch<GPST> : public details::EpochPayload<Epoch<GPST>> {
  typedef std::chrono::gps_clock clock_type;
  typedef std::chrono::duration<long, std::ratio<1, 1000000000>> duration_type;
  typedef std::chrono::time_point<clock_type, duration_type> time_point_type;

  time_point_type tp;

  // default constructor
  constexpr Epoch() = default;
  constexpr Epoch(long t) noexcept : tp(duration_type(t)) {}
  constexpr Epoch(const time_point_type& _tp) : tp(_tp) {}
  constexpr Epoch(time_point_type&& _tp) : tp(std::move(_tp)) {}
  constexpr Epoch& operator=(const time_point_type& _tp) {
    if (&this->tp == &_tp) {
      return *this;
    } else {
      this->tp = _tp;
      return *this;
    }
  }
  constexpr Epoch& operator=(time_point_type&& _tp) {
    if (&this->tp == &_tp) {
      return *this;
    } else {
      this->tp = std::move(_tp);
      return *this;
    }
  }
  ~Epoch() = default;
  constexpr explicit Epoch(const duration_type& dur) : tp(dur) {}

  // Interacting with the ginan library's gTime type
  Epoch(utils::GTime gtime) noexcept;
  operator utils::GTime() const noexcept;

  // reflect constructor
  Epoch(const ReflectionType& _impl);
};

template <>
struct NAVP_EXPORT Epoch<BDT> : public details::EpochPayload<Epoch<BDT>> {
  typedef details::bds_clock clock_type;
  typedef std::chrono::duration<long, std::ratio<1, 1000000000>> duration_type;
  typedef std::chrono::time_point<clock_type, duration_type> time_point_type;

  time_point_type tp;

  // default constructor
  constexpr Epoch() = default;
  constexpr Epoch(long t) noexcept : tp(duration_type(t)) {}
  constexpr Epoch(const time_point_type& _tp) : tp(_tp) {}
  constexpr Epoch(time_point_type&& _tp) : tp(std::move(_tp)) {}
  constexpr Epoch& operator=(const time_point_type& _tp) {
    if (&this->tp == &_tp) {
      return *this;
    } else {
      this->tp = _tp;
      return *this;
    }
  }
  constexpr Epoch& operator=(time_point_type&& _tp) {
    if (&this->tp == &_tp) {
      return *this;
    } else {
      this->tp = std::move(_tp);
      return *this;
    }
  }
  ~Epoch() = default;
  constexpr explicit Epoch(const duration_type& dur) : tp(dur) {}

  // Interacting with the ginan library's gTime type
  Epoch(utils::GTime gtime) noexcept;
  operator utils::GTime() const noexcept;

  // reflect constructor
  Epoch(const ReflectionType& _impl);
};

// operators
template <TimeScaleEnum T>
constexpr auto operator==(const Epoch<T>& lhs, const Epoch<T>& rhs) {
  return lhs.tp == rhs.tp;
}
template <TimeScaleEnum T>
constexpr auto operator<=>(const Epoch<T>& lhs, const Epoch<T>& rhs) {
  return lhs.tp <=> rhs.tp;
}

// export
typedef Epoch<UTC> EpochUtc;
typedef Epoch<GPST> EpochGpst;
typedef Epoch<BDT> EpochBdt;

}  // namespace navp

// std::hash
template <navp::TimeScaleEnum T>
struct NAVP_EXPORT std::hash<navp::Epoch<T>> {
  std::size_t operator()(const navp::Epoch<T>& d) const noexcept { return d.tp.time_since_epoch().count(); }
};

// std::formatter
template <typename CharT>
struct NAVP_EXPORT std::formatter<navp::Epoch<navp::GPST>, CharT> : __format::__formatter_chrono<CharT> {
  using _Duration = navp::Epoch<navp::GPST>::duration_type;

  template <typename _ParseContext>
  constexpr typename _ParseContext::iterator parse(_ParseContext& __pc) {
    return _M_f._M_parse(__pc, __format::_ZonedDateTime);
  }

  template <typename _FormatContext>
  typename _FormatContext::iterator format(const navp::Epoch<navp::GPST>& epoch, _FormatContext& __fc) const {
    // Convert to __local_time_fmt with abbrev "GPS" and offset 0s.
    // Offset is 1980y/January/Sunday[1] - 1970y/January/1
    constexpr chrono::days __gps_offset = chrono::days(3657);
    using _CDur = common_type_t<_Duration, chrono::days>;
    chrono::local_time<_CDur> __lt(epoch.tp.time_since_epoch() + __gps_offset);
    const string __abbrev("GPS", 3);
    const chrono::seconds __off = 0s;
    const auto __lf = chrono::local_time_format(__lt, &__abbrev, &__off);
    return _M_f._M_format(__lf, __fc);
  }

 private:
  __format::__formatter_chrono<CharT> _M_f;
};
template <typename CharT>
struct NAVP_EXPORT std::formatter<navp::Epoch<navp::UTC>, CharT> : __format::__formatter_chrono<CharT> {
  using _Duration = navp::Epoch<navp::UTC>::duration_type;

  template <typename _ParseContext>
  constexpr typename _ParseContext::iterator parse(_ParseContext& __pc) {
    return _M_f._M_parse(__pc, __format::_ZonedDateTime);
  }

  template <typename _FormatContext>
  typename _FormatContext::iterator format(const navp::Epoch<navp::UTC>& epoch, _FormatContext& __fc) const {
    // Adjust by removing leap seconds to get equivalent sys_time.
    // We can't just use clock_cast because we want to know if the time
    // falls within a leap second insertion, and format seconds as "60".
    using chrono::seconds;
    using chrono::sys_time;
    using chrono::__detail::__utc_leap_second;
    using _CDur = common_type_t<_Duration, seconds>;
    const auto __li = chrono::get_leap_second_info(epoch.tp);
    sys_time<_CDur> __s{epoch.tp.time_since_epoch() - __li.elapsed};
    if (!__li.is_leap_second) [[likely]]
      return _M_f._M_format(__s, __fc);
    else
      return _M_f._M_format(__utc_leap_second(__s), __fc);
  }

 private:
  friend formatter<chrono::__detail::__utc_leap_second<_Duration>, CharT>;
  __format::__formatter_chrono<CharT> _M_f;
};
template <typename CharT>
struct NAVP_EXPORT std::formatter<navp::Epoch<navp::BDT>, CharT> : __format::__formatter_chrono<CharT> {
  using _Duration = navp::Epoch<navp::BDT>::duration_type;

  template <typename _ParseContext>
  constexpr typename _ParseContext::iterator parse(_ParseContext& __pc) {
    return _M_f._M_parse(__pc, __format::_ZonedDateTime);
  }

  template <typename _FormatContext>
  typename _FormatContext::iterator format(const navp::Epoch<navp::BDT>& __t, _FormatContext& __fc) const {
    constexpr chrono::days __bds_offset = chrono::days(3657 + 9492);
    using _CDur = common_type_t<_Duration, chrono::days>;
    chrono::local_time<_CDur> __lt(__t.tp.time_since_epoch() + __bds_offset);
    const string __abbrev("BDS", 3);
    const chrono::seconds __off = 0s;
    const auto __lf = chrono::local_time_format(__lt, &__abbrev, &__off);
    return _M_f._M_format(__lf, __fc);
  }

 private:
  __format::__formatter_chrono<CharT> _M_f;
};