#include "magic_enum.hpp"
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "cpptrace/cpptrace.hpp"
#include "doctest.h"
#include "solution/config.hpp"

TEST_CASE("config") {
  cpptrace::register_terminate_handler();
  typedef navp::solution::NavConfigManger::CodeTypeMap CodeType;
  auto cfg_path = "/root/project/nav_cxx/config/rtk_config.toml";
  auto cfg = navp::solution::NavConfigManger(cfg_path);
  auto enabled_codes = cfg.enabled_code();
  if (enabled_codes.is_err()) {
    throw enabled_codes.unwrap_err_unchecked();
  } else {
    auto codes = enabled_codes.unwrap_unchecked();
    for (auto [sys, code] : codes) {
      std::cout << magic_enum::enum_name(sys) << '\t';
      for (auto c : code) {
        std::cout << magic_enum::enum_name(c) << '\t';
      }
      std::cout << std::endl;
    }
  }

  std::cout << "Parse enabled_code done!\n";
}