#include <chrono>
#include <print>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <thread>

#include "doctest.h"
#include "utils/time.hpp"

using namespace navp;
using namespace std::chrono;
using namespace navp::details;

TEST_CASE("clock") {
  constexpr Date bds_begin_epoch = Date{
      .year = 2006,
      .month = 1,
      .day = 1,
  };
  EpochUtc EpochUtc0;
  EpochUtc EpochUtc1 = EpochUtc::from_date(bds_begin_epoch);
  auto dur = (EpochUtc1 - EpochUtc0).seconds();
  CHECK(dur == 1136073623);
}

TEST_CASE("now") {
  std::println("UTC date now : {}", EpochUtc::now());
  std::println("SYS date now : {}", EpochSys::now());
  std::println("TAI date now : {}", EpochTai::now());
  std::println("GPS date now : {}", EpochGps::now());
  std::println("BDS date now : {}", EpochBds::now());
}

TEST_CASE("constructors") {
  // default
  constexpr EpochSys EpochSys0;
  constexpr EpochUtc EpochUtc0;
  constexpr EpochTai EpochTai0;
  constexpr EpochGps EpochGps0;
  constexpr EpochBds EpochBds0;
  std::println("Default EpochSys date : {}", EpochSys0);
  std::println("Default EpochUtc date : {}", EpochUtc0);
  std::println("Default EpochTai date : {}", EpochTai0);
  std::println("Default EpochGps date : {}", EpochGps0);
  std::println("Default EpochBds date : {}", EpochBds0);

  // create a gps time
  constexpr auto gps_time = GpsTime{
      .weeks = 2000,
      .integer_second = 4000,
      .decimal_second = 0.3,
  };

  // gps time constructor
  EpochGps EpochGps1 = EpochGps::from_gps_time(gps_time);
  EpochBds EpochBds1 = EpochBds::from_gps_time(gps_time);
  auto gps_time_info = EpochGps1.gps_time();
  CHECK(gps_time_info.weeks == 2000);
  CHECK(gps_time_info.decimal_second == (f128)0.3);
  auto bds_time_info = EpochBds1.gps_time();
  CHECK(bds_time_info.weeks == 2000);
  CHECK(bds_time_info.integer_second == (f128)4000);

  // create a utc date
  constexpr Date date0 = Date{
      .year = 2018,
      .month = 5,
      .day = 6,
      .hour = 1,
      .minute = 40,
      .integer_second = 22,
      .decimal_second = 0.001,
  };

  EpochUtc EpochUtc2 = EpochUtc::from_date(date0);
  auto date0_ = EpochUtc2.date();
  CHECK(date0_.year == date0.year);
  CHECK(date0_.month == date0.month);
  CHECK(date0_.day == date0.day);
  CHECK(date0_.hour == date0.hour);
  CHECK(date0_.minute == date0.minute);
  CHECK(date0_.integer_second == date0.integer_second);
  CHECK(date0_.decimal_second == date0.decimal_second);

  // create a utc date with time zone local offset
  Date date1 = Date{
      .year = 2018,
      .month = 5,
      .day = 6,
      .hour = 1,
      .minute = 40,
      .integer_second = 22,
      .seconds_offset = TimeZoneOffset::local_offset(),
      .decimal_second = 0.001,
  };

  EpochUtc EpochUtc3 = EpochUtc::from_date(date1);
  // note: no time zone offset parameter means using passing zero time zone
  // if runs `auto date1_ = EpochUtc3.date();`
  // the date_1's hour、day and seconds_offset may be different from date1's,cause they may be at different time zone.
  auto date1_ = EpochUtc3.date(TimeZoneOffset::local_offset());
  CHECK(date1_.year == date1.year);
  CHECK(date1_.month == date1.month);
  CHECK(date1_.day == date1.day);
  CHECK(date1_.hour == date1.hour);
  CHECK(date1_.minute == date1.minute);
  CHECK(date1_.integer_second == date1.integer_second);
  CHECK(date1_.decimal_second == date1.decimal_second);
  CHECK(date1_.seconds_offset == date1.seconds_offset);

  EpochGps gps0 = EpochGps::from_date(date0);
}

TEST_CASE("operator") {
  using namespace std::literals;
  auto date = Date{
      .year = 2024,
      .month = 1,
      .day = 1,
  };
  auto EpochUtc0 = EpochUtc::from_date(date);
  auto EpochUtc1 = EpochUtc0 + 0.5s;

  auto diff_mills = (EpochUtc1 - EpochUtc0).fractional_milliseconds();
  CHECK(diff_mills == 500);
  auto diff_second = (EpochUtc1 - EpochUtc0).scale_fractional_seconds();
  CHECK(diff_second == 0.5);
}

TEST_CASE("hash") {
  EpochUtc EpochUtc0 = EpochUtc::now();
  std::map<EpochUtc, int> mp;
  mp.insert({EpochUtc0, 20});
  CHECK(mp.at(EpochUtc0) == 20);
}

TEST_CASE("formatter") {
  EpochGps EpochGps0;
  std::println("{:%Y-%m-%d %H:%M:%S}\n", EpochGps0);
}

TEST_CASE("parse") {
  auto EpochUtc = Epoch<gps_clock>::from_str("%Y-%m-%d %H:%M:%S", "2021-11-14 07:00:00").unwrap();
  std::println("{}", EpochUtc);
}
