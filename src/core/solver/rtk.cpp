#include "solution/rtk.hpp"

int main() {
  using namespace navp::solution;

  auto rtk = Rtk("/root/project/nav_cxx/config/rtk_config.toml");
  rtk.export_config();
  return 0;
}