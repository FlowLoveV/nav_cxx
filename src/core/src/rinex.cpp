#include "io/rinex.hpp"

namespace navp::io::rinex {

Rinex::Rinex(std::istream& inputstream) {
  allocate_all_members();
  readRnx(inputstream, this->cmn_info->type, *obs, *nav, this->obs_info->header, this->cmn_info->version,
          this->cmn_info->sys, this->obs_info->tsys, sys_code_types);
}

Rinex::Rinex(const char* path) {
  std::ifstream inputstream(path);
  allocate_all_members();
  readRnx(inputstream, this->cmn_info->type, *obs, *nav, this->obs_info->header, this->cmn_info->version,
          this->cmn_info->sys, this->obs_info->tsys, sys_code_types);
}

Rinex::operator GnssObs() && noexcept {
  return GnssObs{
      std::move(this->obs),
      std::move(this->sys_code_types),
      std::move(this->cmn_info),
      std::move(this->obs_info),
  };
}

Rinex::operator GnssNav() && noexcept {
  return GnssNav{
      std::move(this->nav),
      std::move(this->cmn_info),
  };
}

Rinex::operator std::unique_ptr<GnssObs>() && noexcept {
  return static_cast<std::unique_ptr<GnssObs>>(GnssObs{
      std::move(this->obs),
      std::move(this->sys_code_types),
      std::move(this->cmn_info),
      std::move(this->obs_info),
  });
}

Rinex::operator std::unique_ptr<GnssNav>() && noexcept {
  return static_cast<std::unique_ptr<GnssNav>>(GnssNav{
      std::move(this->nav),
      std::move(this->cmn_info),
  });
}

GnssObs::GnssObs(std::unique_ptr<ObsList>&& obs_list, std::map<E_Sys, std::map<int, CodeType>>&& sys_code_types,
                 std::unique_ptr<RinexCommonInfo>&& cmn_info, std::unique_ptr<RinexObsInfo>&& obs_info)
    : obs_list(std::move(obs_list)),
      sys_code_types(std::move(sys_code_types)),
      cmn_info(std::move(cmn_info)),
      obs_info(std::move(obs_info)) {}

GnssObs::GnssObs(std::istream& inputstream) {
  auto rinex = Rinex(inputstream);
  *this = std::move(static_cast<GnssObs>(std::move(rinex)));
}

GnssObs::GnssObs(const char* path) {
  auto rinex = Rinex(path);
  *this = std::move(static_cast<GnssObs>(std::move(rinex)));
}

GnssObs::operator std::unique_ptr<GnssObs>() && noexcept { return std::make_unique<GnssObs>(std::move(*this)); }

GnssNav::GnssNav(std::unique_ptr<Navigation>&& nav, std::unique_ptr<RinexCommonInfo>&& info)
    : nav(std::move(nav)), info(std::move(info)) {}

GnssNav::GnssNav(std::istream& inputstream) {
  auto rinex = Rinex(inputstream);
  *this = std::move(static_cast<GnssNav>(std::move(rinex)));
}

GnssNav::GnssNav(const char* path) {
  auto rinex = Rinex(path);
  *this = std::move(static_cast<GnssNav>(std::move(rinex)));
}

GnssNav::operator std::unique_ptr<GnssNav>() && noexcept { return std::make_unique<GnssNav>(std::move(*this)); }

}  // namespace navp::io::rinex
