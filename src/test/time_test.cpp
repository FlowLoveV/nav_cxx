#include <chrono>
#include <map>
#include <print>

#include "utils/macro.hpp"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "rfl/json.hpp"
#include "utils/gTime.hpp"
#include "utils/time.hpp"

using namespace navp;
using namespace std::chrono;
using namespace navp::details;
typedef Epoch<UTC> Utc;
typedef Epoch<GPST> Gpst;
typedef Epoch<BDT> Bdt;

TEST_CASE("constructors") {
  // default
  Utc utc0;
  Gpst gpst0;
  Bdt bdt0;
  NAV_PRINT("Default UTC {}\nDefault GPST {}\nDefault BDT {}\n", utc0, gpst0, bdt0);

  // gps time constructor
  Gpst gpst1 = Gpst::from_gps_time(2000, 6040);
  Bdt bdt1 = Bdt::from_gps_time(800, 2004, 3e8);
  auto gps_time_info = gpst1.gps_time();
  CHECK(std::get<0>(gps_time_info) == 2000);
  CHECK(std::get<1>(gps_time_info) == 6040);
  auto bds_time_info = bdt1.gps_time();
  CHECK(std::get<0>(bds_time_info) == 800);
  CHECK(std::get<1>(bds_time_info) == 2004.3);

  // utc constructor
  Utc utc1 = Utc::from_local_date(2018, 5, 6, 1, 40, 22.001);
  auto [y1, m1, d1, H1, M1, S1, N1] = utc1.local_date();
  CHECK(y1 == 2018);
  CHECK(m1 == 5);
  CHECK(d1 == 6);
  CHECK(H1 == 1);
  CHECK(M1 == 40);
  CHECK(S1 == 22);
  CHECK(N1 == 1E6);
  Utc utc2 = Utc::from_utc_date(2018, 5, 6, 1, 40, 22, 1E6);
  auto [y2, m2, d2, H2, M2, S2, N2] = utc2.utc_date();
  CHECK(y2 == 2018);
  CHECK(m2 == 5);
  CHECK(d2 == 6);
  CHECK(H2 == 1);
  CHECK(M2 == 40);
  CHECK(S2 == 22);
  CHECK(N2 == 1E6);

  // from str,the str should be expresed in utc style
  // the date in string is utc date
  Utc utc3 = Utc::from_str("%Y-%m-%d %H:%M:%S", "2024-7-23 17:32:30.1").unwrap();
  auto [y3, m3, d3, H3, M3, S3, N3] = utc3.utc_date();
  CHECK(y3 == 2024);
  CHECK(m3 == 7);
  CHECK(d3 == 23);
  CHECK(H3 == 17);
  CHECK(M3 == 32);
  CHECK(S3 == 30);
  CHECK(N3 == 1E8);

  Gpst gpst2 = Gpst::from_str("%Y-%m-%d %H:%M:%S", "2024-7-23 17:32:30.1").unwrap();
  // As of July 23, 2024, GPST is 18 seconds offset from UTC
  CHECK_EQ(NAV_FORMAT("{}", gpst2), "2024-07-23 17:32:48.100000000");
  Bdt bdt2 = Bdt::from_str("%Y-%m-%d %H:%M:%S", "2024-7-23 17:32:30.1").unwrap();
  // As of July 23, 2024, BDT is 4 seconds offset from UTC
  CHECK_EQ(NAV_FORMAT("{}", bdt2), "2024-07-23 17:32:34.100000000");
}

TEST_CASE("cast") {
  Utc utc0 = Utc::now();
  NAV_PRINT("{}\n", utc0);
  NAV_PRINT("{}\n", epoch_cast<GPST>(utc0));
  NAV_PRINT("{}\n", epoch_cast<BDT>(utc0));

  auto utc_tp = utc0.tp;
  NAV_PRINT("{}\n", utc_tp);
  NAV_PRINT("{}\n", clock_cast<gps_clock>(utc_tp));
  NAV_PRINT("{}\n", clock_cast<bds_clock>(utc_tp));
}

TEST_CASE("hash") {
  Utc utc0 = Utc::now();
  std::map<Utc, int> mp;
  mp.insert({utc0, 20});
  CHECK(mp.at(utc0) == 20);
}

TEST_CASE("formatter") {
  Gpst gpst0;
  NAV_PRINT("{:%Y-%m-%d %H:%M:%S}\n", gpst0);
}

TEST_CASE("reflect") {
  auto utc = Epoch<UTC>::now();
  auto json = rfl::json::write(utc);
  std::println("{}", json);
}

TEST_CASE("gtime") {
  auto utc = Epoch<UTC>::from_str("%Y-%m-%d %H:%M:%S", "2021-11-14 07:00:00").unwrap();
  utils::GTime gtime = utc;
  Epoch<UTC> utc_verse(gtime);
  NAV_PRINTLN("{}", utc_verse);
}