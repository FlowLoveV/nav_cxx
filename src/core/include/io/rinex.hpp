#pragma once

#include <memory>

#include "ginan/cpp/common/navigation.hpp"
#include "ginan/cpp/common/observations.hpp"
#include "ginan/cpp/common/receiver.hpp"
#include "ginan/cpp/common/rinex.hpp"

namespace navp::io::rinex {

struct RinexCommonInfo;
struct RinexObsInfo;
class Rinex;
class GnssObs;
class GnssNav;

struct RinexCommonInfo {
  double version;
  E_Sys sys;
  char type;
};

struct RinexObsInfo {
  E_TimeSys tsys;
  RinexStation header;
};

class Rinex {
 public:
  Rinex(std::istream& inputstream);

  Rinex(const char* path);

  operator GnssObs() && noexcept;

  operator GnssNav() && noexcept;

  operator std::unique_ptr<GnssObs>() && noexcept;

  operator std::unique_ptr<GnssNav>() && noexcept;

 private:
  void allocate_all_members() {
    this->cmn_info = std::make_unique<RinexCommonInfo>();
    this->obs_info = std::make_unique<RinexObsInfo>();
    this->obs = std::make_unique<ObsList>();
    this->nav = std::make_unique<Navigation>();
  }

  std::unique_ptr<RinexCommonInfo> cmn_info;
  std::unique_ptr<RinexObsInfo> obs_info;
  std::map<E_Sys, std::map<int, CodeType>> sys_code_types;
  std::unique_ptr<ObsList> obs;
  std::unique_ptr<Navigation> nav;
};

class GnssObs {
 public:
  GnssObs(std::unique_ptr<ObsList>&& obs_list, std::map<E_Sys, std::map<int, CodeType>>&& sys_code_types,
          std::unique_ptr<RinexCommonInfo>&& cmn_info, std::unique_ptr<RinexObsInfo>&& obs_info);

  GnssObs(std::istream& inputstream);

  GnssObs(const char* path);

  operator std::unique_ptr<GnssObs>() && noexcept;

 public:
  std::shared_ptr<RinexCommonInfo> cmn_info;
  std::shared_ptr<RinexObsInfo> obs_info;
  std::shared_ptr<ObsList> obs_list;
  std::map<E_Sys, std::map<int, CodeType>> sys_code_types;
};

class GnssNav {
 public:
  GnssNav(std::unique_ptr<Navigation>&& nav, std::unique_ptr<RinexCommonInfo>&& info);

  GnssNav(std::istream& inpurstream);

  GnssNav(const char* path);

  operator std::unique_ptr<GnssNav>() && noexcept;

 public:
  std::shared_ptr<Navigation> nav;
  std::shared_ptr<RinexCommonInfo> info;
};

}  // namespace navp::io::rinex