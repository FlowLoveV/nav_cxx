#pragma once

#include <istream>
#include <map>

#include "sensors/gnss/enums.hpp"
#include "utils/eigen.hpp"
#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace navp::sensors::gnss {
struct Navigation;
struct ObsList;
}  // namespace navp::sensors::gnss

namespace navp::io::rinex {

using sensors::gnss::ConstellationEnum;
using sensors::gnss::Navigation;
using sensors::gnss::ObsCodeEnum;
using sensors::gnss::ObsList;
using sensors::gnss::TimeSystemEnum;

struct CodeType {
  char type = 0;
  ObsCodeEnum code = ObsCodeEnum::NONE;
};

struct NAVP_EXPORT RinexStation {
  std::string id;                                           ///< marker name
  std::string marker;                                       ///< marker number
  std::string antDesc;                                      ///< antenna descriptor
  std::string antSerial;                                    ///< antenna serial number
  std::string recType;                                      ///< receiver type descriptor
  std::string recFWVersion;                                 ///< receiver firmware version
  std::string recSerial;                                    ///< receiver serial number
  utils::NavVector3f64 del = utils::NavVector3f64::Zero();  ///< antenna position delta (e/n/u or x/y/z) (m)
  utils::NavVector3f64 pos = utils::NavVector3f64::Zero();
};

/// read rinex file header
i32 readRnxH(std::istream& inputStream, f64& ver, char& type, ConstellationEnum& sys, TimeSystemEnum& tsys,
             std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes, Navigation& nav, RinexStation& rnxRec,
             char* glo_fcn, f64* glo_cpbias);

/// read rinex observation at next epoch
i32 readNextRnxObsB(std::istream& inputStream, f64 ver, TimeSystemEnum tsys,
                    std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes, i32& flag, ObsList& obsList);

/// read rinex obsrvation file body
i32 readRnxObs(std::istream& inputStream, f64 ver, TimeSystemEnum tsys,
               std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes, ObsList& obsList,
               RinexStation& rnxRec);

/// read rinex nav/gnav/geo nav
i32 readRnxNav(std::istream& inputStream,  ///< Input stream to read
               f64 ver,                    ///< RINEX version
               ConstellationEnum sys,      ///< Satellite system
               Navigation& nav);

/// read rinex clock file
i32 readRnxClk(std::istream& inputStream, f64 ver, Navigation& nav);

/// read rinex file
i32 readRnx(std::istream& inputStream, char& type, ObsList& obsList, Navigation& nav, RinexStation& rnxRec, f64& ver,
            ConstellationEnum& sys, TimeSystemEnum& tsys,
            std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes, char* glo_fcn, f64* glo_cpbias);

}  // namespace navp::io::rinex