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
  std::map<std::string, std::map<utils::GTime, Pclk>> pclkMap;  ///< precise clock

  std::map<Sv, std::map<NavMsgTypeEnum, std::map<utils::GTime, Eph, std::less<utils::GTime>>>>
      ephMap;  ///< GPS/QZS/GAL/BDS ephemeris
  std::map<Sv, std::map<NavMsgTypeEnum, std::map<utils::GTime, Geph, std::less<utils::GTime>>>>
      gephMap;  ///< GLONASS ephemeris
  std::map<Sv, std::map<NavMsgTypeEnum, std::map<utils::GTime, Seph, std::less<utils::GTime>>>>
      sephMap;  ///< SBAS ephemeris
  std::map<Sv, std::map<NavMsgTypeEnum, std::map<utils::GTime, Ceph, std::less<utils::GTime>>>>
      cephMap;  ///< GPS/QZS/BDS CNVX ephemeris
  std::map<ConstellationEnum, std::map<NavMsgTypeEnum, std::map<utils::GTime, ION, std::less<utils::GTime>>>>
      ionMap;  ///< ION messages
  std::map<StoCodeEnum, std::map<NavMsgTypeEnum, std::map<utils::GTime, STO, std::less<utils::GTime>>>>
      stoMap;  ///< STO messages
  std::map<ConstellationEnum, std::map<NavMsgTypeEnum, std::map<utils::GTime, EOP, std::less<utils::GTime>>>>
      eopMap;  ///< EOP messages

  ginan::ERP erp;       /* earth rotation parameters */
  i32 leaps = -1;       /* leap seconds (s) */
  char glo_fcn[27 + 1]; /* glonass frequency channel number + 8 */
  f64 glo_cpbias[4];    /* glonass code-phase bias {1C,1P,2C,2P} (m) */
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