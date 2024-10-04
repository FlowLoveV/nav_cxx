#include "io/rinex/rinex.hpp"

namespace navp::io::rinex {

Rinex::Rinex(std::istream& inputstream) {
  allocate_all_members();
  details::readRnx(inputstream, this->cmn_info->type, *obs, *nav, this->obs_info->header, this->cmn_info->version,
                   this->cmn_info->sys, this->obs_info->tsys, sys_code_types);
}

Rinex::Rinex(const char* path) {
  std::ifstream inputstream(path);
  allocate_all_members();
  details::readRnx(inputstream, this->cmn_info->type, *obs, *nav, this->obs_info->header, this->cmn_info->version,
                   this->cmn_info->sys, this->obs_info->tsys, sys_code_types);
}

Rinex::operator RinexObs() && noexcept {
  return RinexObs{
      std::move(this->obs),
      std::move(this->sys_code_types),
      std::move(this->cmn_info),
      std::move(this->obs_info),
  };
}

Rinex::operator RinexNav() && noexcept {
  return RinexNav{
      std::move(this->nav),
      std::move(this->cmn_info),
  };
}

Rinex::operator std::unique_ptr<RinexObs>() && noexcept {
  return static_cast<std::unique_ptr<RinexObs>>(RinexObs{
      std::move(this->obs),
      std::move(this->sys_code_types),
      std::move(this->cmn_info),
      std::move(this->obs_info),
  });
}

Rinex::operator std::unique_ptr<RinexNav>() && noexcept {
  return static_cast<std::unique_ptr<RinexNav>>(RinexNav{
      std::move(this->nav),
      std::move(this->cmn_info),
  });
}

void Rinex::allocate_all_members() {
  this->cmn_info = std::make_unique<RinexCommonInfo>();
  this->obs_info = std::make_unique<RinexObsInfo>();
  this->obs = std::make_unique<ObsList>();
  this->nav = std::make_unique<Navigation>();
}

RinexObs::RinexObs(std::unique_ptr<ObsList>&& obs_list,
                   std::map<sensors::gnss::ConstellationEnum, std::map<i32, CodeType>>&& sys_code_types,
                   std::unique_ptr<RinexCommonInfo>&& cmn_info, std::unique_ptr<RinexObsInfo>&& obs_info)
    : obs_list(std::move(obs_list)),
      sys_code_types(std::move(sys_code_types)),
      cmn_info(std::move(cmn_info)),
      obs_info(std::move(obs_info)) {}

RinexObs::RinexObs(std::istream& inputstream) {
  auto rinex = Rinex(inputstream);
  *this = std::move(static_cast<RinexObs>(std::move(rinex)));
}

RinexObs::RinexObs(const char* path) {
  auto rinex = Rinex(path);
  *this = std::move(static_cast<RinexObs>(std::move(rinex)));
}

RinexObs::operator std::unique_ptr<RinexObs>() && noexcept { return std::make_unique<RinexObs>(std::move(*this)); }

RinexObs::operator RecordGnssObs() && noexcept { return RecordGnssObs(std::move(*this).obs_list); }

RinexNav::RinexNav(std::unique_ptr<Navigation>&& nav, std::unique_ptr<RinexCommonInfo>&& info)
    : nav(std::move(nav)), info(std::move(info)) {}

RinexNav::RinexNav(std::istream& inputstream) {
  auto rinex = Rinex(inputstream);
  *this = std::move(static_cast<RinexNav>(std::move(rinex)));
}

RinexNav::RinexNav(const char* path) {
  auto rinex = Rinex(path);
  *this = std::move(static_cast<RinexNav>(std::move(rinex)));
}

RinexNav::operator std::unique_ptr<RinexNav>() && noexcept { return std::make_unique<RinexNav>(std::move(*this)); }

RinexNav::operator RecordGnssNav() && noexcept { return RecordGnssNav(std::move(*this).nav); }

}  // namespace navp::io::rinex
