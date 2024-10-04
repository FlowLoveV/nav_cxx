#pragma once

#include <memory>

#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "utils/eigen.hpp"

namespace navp::io::rinex {

using sensors::gnss::ConstellationEnum;
using sensors::gnss::Navigation;
using sensors::gnss::NavMsgTypeEnum;
using sensors::gnss::ObsCodeEnum;
using sensors::gnss::ObsList;
using sensors::gnss::TimeSystemEnum;

using sensors::gnss::RecordGnssNav;
using sensors::gnss::RecordGnssObs;

struct CodeType {
  char type = 0;
  ObsCodeEnum code = ObsCodeEnum::NONE;
};

struct RinexStation {
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

struct RinexCommonInfo;
struct RinexObsInfo;
class Rinex;
class RinexObs;
class RinexNav;

struct RinexCommonInfo {
  f64 version;
  ConstellationEnum sys;
  char type;
};

struct RinexObsInfo {
  TimeSystemEnum tsys;
  RinexStation header;
};

class Rinex {
 public:
  // construct from inputstream(obs stream or nav stream)
  Rinex(std::istream& inputstream);
  // construct from a single rinex file(obs or nav)
  Rinex(const char* path);

  // Convert to GnssObs type, move semantics
  operator RinexObs() && noexcept;
  operator std::unique_ptr<RinexObs>() && noexcept;

  // Convert to GnssNav type, move semantics
  operator RinexNav() && noexcept;
  operator std::unique_ptr<RinexNav>() && noexcept;

 private:
  void allocate_all_members();

  std::unique_ptr<RinexCommonInfo> cmn_info;
  std::unique_ptr<RinexObsInfo> obs_info;
  std::map<ConstellationEnum, std::map<i32, CodeType>> sys_code_types;
  std::unique_ptr<ObsList> obs;
  std::unique_ptr<Navigation> nav;
};

class RinexObs {
 public:
  RinexObs(std::unique_ptr<ObsList>&& obs_list, std::map<ConstellationEnum, std::map<i32, CodeType>>&& sys_code_types,
           std::unique_ptr<RinexCommonInfo>&& cmn_info, std::unique_ptr<RinexObsInfo>&& obs_info);

  RinexObs(std::istream& inputstream);

  RinexObs(const char* path);

  operator std::unique_ptr<RinexObs>() && noexcept;

  operator RecordGnssObs() && noexcept;

 public:
  std::unique_ptr<RinexCommonInfo> cmn_info;
  std::unique_ptr<RinexObsInfo> obs_info;
  std::unique_ptr<ObsList> obs_list;
  std::map<ConstellationEnum, std::map<i32, CodeType>> sys_code_types;
};

class RinexNav {
 public:
  RinexNav(std::unique_ptr<Navigation>&& nav, std::unique_ptr<RinexCommonInfo>&& info);

  RinexNav(std::istream& inpurstream);

  RinexNav(const char* path);

  operator std::unique_ptr<RinexNav>() && noexcept;

  operator RecordGnssNav() && noexcept;

 public:
  std::unique_ptr<Navigation> nav;
  std::unique_ptr<RinexCommonInfo> info;
};

namespace details {
i32 readRnx(std::istream& inputStream, char& type, ObsList& obsList, Navigation& nav, RinexStation& rnxRec, f64& ver,
            ConstellationEnum& sys, TimeSystemEnum& tsys,
            std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes);
}

}  // namespace navp::io::rinex