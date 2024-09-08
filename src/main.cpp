#include <fstream>
#include <print>

#include "ginan/cpp/common/navigation.hpp"
#include "ginan/cpp/common/rinex.hpp"
#include "ginan/cpp/common/streamRinex.hpp"
#include "logger.hpp"
#include "macro.hpp"
#include "rfl.hpp"
#include "rfl/json.hpp"
#include "sensors/gnss/sv.hpp"

int main(int argc, char *argv[]) {
  using namespace navp;
  using namespace navp::sensors::gnss;
  for (int i = 1; i < argc; i++) {
    std::ifstream input(argv[i]);
    char type;
    ObsList obs;
    Navigation nav;
    RinexStation rnxRec;
    double ver;
    E_Sys sys;
    E_TimeSys tsys;
    std::map<E_Sys, map<int, CodeType>> sysCodeTypes;
    readRnx(input, type, obs, nav, rnxRec, ver, sys, tsys, sysCodeTypes);

    const std::string json_string = rfl::json::write(Sv{20,ConstellationEnum::GPS});
    NAV_PRINT("{}", json_string);
    nav_debug("reading file {}, version = {}", argv[i], ver);
  }
  return 0;
}