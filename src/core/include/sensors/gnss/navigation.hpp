#pragma once

#include "ginan/erp.hpp"
#include "io/record.hpp"
#include "sensors/gnss/ephemeris.hpp"
#include "utils/gTime.hpp"
#include "utils/macro.hpp"

namespace navp::sensors::gnss {

class NAVP_EXPORT GnssNavRecord;

/** navigation data type
 */
struct NAVP_EXPORT Navigation {
  // clang-format off
  using PclkMapType = std::map<std::string, std::map<utils::GTime, Pclk>>;  // todo, may need changing the map key
  using PephMapType = std::unordered_map<Sv, std::map<utils::GTime, Peph>>;
  using EphMapType = std::unordered_map<Sv, std::unordered_map<NavMsgTypeEnum, std::map<utils::GTime, Eph, std::less<utils::GTime>>>>;
  using GephMapType = std::unordered_map<Sv, std::unordered_map<NavMsgTypeEnum, std::map<utils::GTime, Geph, std::less<utils::GTime>>>>;
  using SephMapType = std::unordered_map<Sv, std::unordered_map<NavMsgTypeEnum, std::map<utils::GTime, Seph, std::less<utils::GTime>>>>;
  using CephMapType = std::unordered_map<Sv, std::unordered_map<NavMsgTypeEnum, std::map<utils::GTime, Ceph, std::less<utils::GTime>>>>;
  using IonMapType = std::unordered_map<ConstellationEnum, std::unordered_map<NavMsgTypeEnum, std::map<utils::GTime, ION, std::less<utils::GTime>>>>;
  using StoMapType = std::unordered_map<StoCodeEnum, std::unordered_map<NavMsgTypeEnum, std::map<utils::GTime, STO, std::less<utils::GTime>>>>;
  using EopMapType = std::unordered_map<ConstellationEnum, std::unordered_map<NavMsgTypeEnum, std::map<utils::GTime, EOP, std::less<utils::GTime>>>>;
  // clang-format on
  PclkMapType pclkMap;  ///< precise clock
  PephMapType pephMap;  ///< precise ephemeris
  EphMapType ephMap;    ///< GPS/QZS/GAL/BDS ephemeris
  GephMapType gephMap;  ///< GLONASS ephemeris
  SephMapType sephMap;  ///< SBAS ephemeris
  CephMapType cephMap;  ///< GPS/QZS/BDS CNVX ephemeris
  IonMapType ionMap;    ///< ION messages
  StoMapType stoMap;    ///< STO messages
  EopMapType eopMap;    ///< EOP messages

  ginan::ERP erp; /* earth rotation parameters */
};

class GnssNavRecord : public io::Record {
 public:
  GnssNavRecord();
  GnssNavRecord(Navigation&& _nav) noexcept;
  GnssNavRecord(std::unique_ptr<Navigation>&& _nav_ptr) noexcept;

  virtual ~GnssNavRecord() override;

  std::shared_ptr<Navigation> nav;
};

}  // namespace navp::sensors::gnss