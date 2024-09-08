#include <print>

#include "ginan/cpp/common/navigation.hpp"
#include "ginan/cpp/common/rinex.hpp"
#include "ginan/cpp/common/streamRinex.hpp"
#include "macro.hpp"
#include "space_time/time.hpp"

int main() {
  std::ifstream input("/root/project/nav_cxx/test_resources/MET/V2/abvi0010.15m");
  char type;
  ObsList obs;
  Navigation nav;
  RinexStation rnxRec;
  double ver;
  E_Sys sys;
  E_TimeSys tsys;
  std::map<E_Sys, map<int, CodeType>> sysCodeTypes;
  readRnx(input, type, obs, nav, rnxRec, ver, sys, tsys, sysCodeTypes);
  // auto gtime = obs.front()->time;
  // navp::Epoch<nav::UTC> gpst = gtime;
  // NAV_PRINTLN("{}", gpst);
  return 0;
}