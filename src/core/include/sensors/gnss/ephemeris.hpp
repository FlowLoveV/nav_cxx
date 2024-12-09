#pragma once

#include <Eigen/Eigen>

#include "enums.hpp"
#include "sv.hpp"
#include "utils/gTime.hpp"
#include "utils/eigen.hpp"
#include "utils/macro.hpp"

namespace navp::sensors::gnss {

struct NAVP_EXPORT KeplerEph {
  f64 A = 0;     ///< semi major axis
  f64 e = 0;     ///< eccentricity
  f64 i0 = 0;    ///< inclination
  f64 OMG0 = 0;  ///< right ascension of ascending node
  f64 omg = 0;   ///< argument of perigee
  f64 M0 = 0;    ///< mean anomoly
  f64 deln = 0;  ///< correction mean motion
  f64 OMGd = 0;  ///< rate of OMG
  f64 idot = 0;  ///< rate of inclination
  f64 crc = 0;   ///< correction radial		cosine
  f64 crs = 0;   ///< correction radial		  sine
  f64 cuc = 0;   ///< correction lattitude	cosine
  f64 cus = 0;   ///< correction lattitude	  sine
  f64 cic = 0;   ///< correction inclination cosine
  f64 cis = 0;   ///< correction inclination   sine
  f64 dn0d = 0;  ///< rate of correction mean motion
  f64 Adot = 0;  ///< rate of A
};

struct NAVP_EXPORT BrdcEph {};

struct NAVP_EXPORT Eph : BrdcEph, KeplerEph {
  NavMsgTypeEnum type = NavMsgTypeEnum::NONE;  ///< message type
  Sv Sat;                                      ///< satellite number
  i32 iode = -1;                               ///< GPS/QZS: IODE, GAL: IODnav
  i32 iodc = 0;                                ///< IODC
  i32 aode;                                    ///< BDS AODE
  i32 aodc;                                    ///< BDS AODC
  i32 sva;                                     ///< SV accuracy (URA index)
  SvhEnum svh;                                 ///< SV health
  i32 week;          ///< GPS/QZS: gps week, GAL:gps week (i.e. galileo week + 1024), BDS: beidou week
  i32 code = 0;      ///< GPS/QZS: code on L2, GAL: data source
  i32 flag = 0;      ///< GPS L2 P data flag
  i32 howTow;        ///< Hand over word time
  utils::GTime toc;  ///< time of clock
  utils::GTime toe;  ///< time of ephemeris
  utils::GTime ttm;  ///< transmission time

  f64 toes;         ///< TOE (s) in week
  f64 fit;          ///< fit interval (h)
  f64 f0;           ///< SV clock parameter (af0)
  f64 f1;           ///< SV clock parameter (af1)
  f64 f2;           ///< SV clock parameter (af2)
  f64 tgd[4] = {};  ///< group delay parameters
                       ///< GPS/QZS:tgd[0]=TGD
                       ///< GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1
                       ///< BDS    :tgd[0]=BGD1,tgd[1]=BGD2

  SatTypeEnum orb = SatTypeEnum::NONE;  ///< BDS sat/orbit type
  utils::GTime top = {};                ///< time of prediction
  f64 tops = 0;                      ///< t_op (s) in week
  f64 ura[4] = {};                   ///< user range accuracy or GAL SISA
                       ///< GPS/QZS CNVX: ura[0]=URAI_NED0, ura[1]=URAI_NED1, ura[2]=URAI_NED2, ura[3]=URAI_ED
  f64 isc[6] =
      {};  ///< inter-signal corrections
           ///< GPS/QZS CNAV: isc[0]=ISC_L1CA, isc[1]=ISC_L2C, isc[2]=ISC_L5I5, isc[3]=ISC_L5Q5
           ///< GPS/QZS CNV2: isc[0]=ISC_L1CA, isc[1]=ISC_L2C, isc[2]=ISC_L5I5, isc[3]=ISC_L5Q5, isc[4]=ISC_L1Cd,
           ///< isc[5]=ISC_L1Cp BDS	 CNV1: isc[0]=ISC_B1Cd BDS	 CNV2: isc[1]=ISC_B2ad
  f64 sis[5] =
      {};  ///< signal in space accuracy index
           ///< BDS CNVX sis[0]=SISAI_oe, sis[1]=SISAI_ocb, sis[2]=SISAI_oc1, sis[3]=SISAI_oc2, sis[4]=SISMAI

  // original messages from stream/rinex for debugging
  f64 tocs;       ///< TOC (s) within week
  i32 weekRollOver;  ///< week number (rolled over)
  f64 sqrtA;      ///< sqrt A
  i32 e5a_hs = 0;    ///< GAL E5a signal health status
  i32 e5a_dvs = 0;   ///< GAL E5a data validity status
  i32 e5b_hs = 0;    ///< GAL E5b signal health status
  i32 e5b_dvs = 0;   ///< GAL E5b data validity status
  i32 e1_hs = 0;     ///< GAL E1 signal health status
  i32 e1_dvs = 0;    ///< GAL E1 data validity status
  f64 ttms = 0;   ///< transmission time (s) within week
  i32 fitFlag = 0;   ///< fit flag
};

struct NAVP_EXPORT Geph : BrdcEph {
  NavMsgTypeEnum type = NavMsgTypeEnum::NONE;  ///< message type
  Sv Sat;                                      ///< satellite number
  i32 iode = -1;                               ///< IODE (0-6 bit of tb field)
  i32 frq;                                     ///< satellite frequency number
  SvhEnum svh;                                 ///< satellite health
  i32 sva;                                     ///< satellite accuracy
  i32 age;                                     ///< age of operation
  utils::GTime toe;                            ///< epoch of epherides (gpst)
  utils::GTime tof;                            ///< message frame time (gpst)
  utils::NavVector3f64 pos;                         ///< satellite position (ecef) (m)
  utils::NavVector3f64 vel;                         ///< satellite velocity (ecef) (m/s)
  utils::NavVector3f64 acc;                         ///< satellite acceleration (ecef) (m/s^2)
  f64 taun;                                 ///< SV clock bias (s)
  f64 gammaN;                               ///< SV relative freq bias
  f64 dtaun;                                ///< delay between L1 and L2 (s)

  // original messages from stream/rinex for debugging
  f64 tofs;    ///< TOF (s) within the current day
  i32 tk_hour;    ///< number of hours of TOF
  i32 tk_min;     ///< number of minutes of TOF
  f64 tk_sec;  ///< seconds of TOF
  i32 tb;         ///< number of 15 min of TOE
  i32 glonassM;   ///< type of GLO satellites
  i32 NT;         ///< calender number of day within 4-year interval
  bool moreData;  ///< availability of additional data
  i32 N4;         ///< 4-year interval number
};

struct NAVP_EXPORT Pclk {
  f64 clk = 999999.999999;  ///< satellite clock (s)
  f64 clkStd = 0;           ///< satellite clock std (s)
  i32 clkIndex;                ///< clock index for multiple files
};

/** precise ephemeris
 */
struct NAVP_EXPORT Peph : Pclk {
  Sv Sat;                                            ///< satellite number
  utils::GTime time;                                 ///< time (GPST)
  i32 index;                                         ///< ephemeris index for multiple files
  utils::NavVector3f64 pos;                               ///< satellite position					(m)
  utils::NavVector3f64 posStd = utils::NavVector3f64::Zero();  ///< satellite position std				(m)
  utils::NavVector3f64 vel;                               ///< satellite velocity/clk-rate		(m/s)
  utils::NavVector3f64 velStd = utils::NavVector3f64::Zero();  ///< satellite velocity/clk-rate std	(m/s)
};

/** Satellite attitude
 */
struct NAVP_EXPORT Att {
  std::string id;
  utils::GTime time;  ///< time (GPST)
  i32 index;          ///< ephemeris index for multiple files
  ObxFrameEnum frame;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  ///< satellite attitude represented w/ a quaternion
};

/** SBAS ephemeris
 */
struct NAVP_EXPORT Seph : BrdcEph {
  NavMsgTypeEnum type = NavMsgTypeEnum::NONE;  ///< message type
  Sv Sat;                                      ///< satellite number
  utils::GTime t0;                             ///< reference epoch time (GPST)
  utils::GTime tof;                            ///< time of message frame (GPST)
  i32 sva;                                     ///< SV accuracy (URA index)
  SvhEnum svh;                                 ///< SV health
  utils::NavVector3f64 pos;                         ///< satellite position (m) (ecef)
  utils::NavVector3f64 vel;                         ///< satellite velocity (m/s) (ecef)
  utils::NavVector3f64 acc;                         ///< satellite acceleration (m/s^2) (ecef)
  f64 af0 = 0;                              ///< satellite clock-offset/drift (s)
  f64 af1 = 0;                              ///< satellite clock-drift (s/s)
  i32 iode = -1;                               // unused, for templating only
  utils::GTime toe;                            // unused, for templating only

  f64 tofs;  ///< TOF (s) within the week
};

/** GPS/QZS CNAV/CNAV-2 or BDS CNAV-1/CNAV-2/CNAV-3 ephemeris
 */
struct NAVP_EXPORT Ceph : KeplerEph {
  NavMsgTypeEnum type = NavMsgTypeEnum::NONE;  ///< message type
  SatTypeEnum orb = SatTypeEnum::NONE;         ///< BDS sat/orbit type
  Sv Sat;                                      ///< satellite number
  i32 iode = -1;                               ///< BDS CNAV1/CNV2 IODE
  i32 iodc = -1;                               ///< BDS CNAV1/CNV2 IODC
  SvhEnum svh;                                 ///< SV health
  i32 wnop = 0;                                ///< GPS/QZS: GPS week number (of prediction?) with AR
  i32 flag = 0;                                ///< BDS B1C/B2a+B1C/B2b integrity flags
  utils::GTime toc = {};                       ///< time of clock
  utils::GTime toe = {};                       ///< time of ephemeris, for GPS/QZS, TOE==TOC
  utils::GTime top = {};                       ///< time of prediction
  utils::GTime ttm = {};                       ///< transmission time

  f64 ura[4] = {};  ///< user range accuracy
                       ///< GPS/QZS: ura[0]=URAI_NED0, ura[1]=URAI_NED1, ura[2]=URAI_NED2, ura[3]=URAI_ED
  f64 isc[6] =
      {};  ///< inter-signal corrections 
           ///< GPS/QZS CNAV: isc[0]=ISC_L1CA, isc[1]=ISC_L2C, isc[2]=ISC_L5I5, isc[3]=ISC_L5Q5
           ///< GPS/QZS CNV2: isc[0]=ISC_L1CA, isc[1]=ISC_L2C, isc[2]=ISC_L5I5, isc[3]=ISC_L5Q5, isc[4]=ISC_L1Cd,
           ///< isc[5]=ISC_L1Cp BDS	 CNV1: isc[0]=ISC_B1Cd BDS	 CNV2: isc[1]=ISC_B2ad
  f64 sis[5] = {};  ///< signal in space accuracy index
                       ///< BDS sis[0]=SISAI_oe, sis[1]=SISAI_ocb, sis[2]=SISAI_oc1, sis[3]=SISAI_oc2, sis[4]=SISMAI
  f64 tops = 0;     ///< t_op (s) in week
  f64 toes = 0;     ///< TOE (s) in week
  f64 f0 = 0;       ///< SV clock parameter (af0)
  f64 f1 = 0;       ///< SV clock parameter (af1)
  f64 f2 = 0;       ///< SV clock parameter (af2)
  f64 tgd[4] = {};  ///< group delay parameters
                       ///< GPS/QZS:		tgd[0]=TGD
                       ///< BDS CNAV1/CNV2: tgd[0]=TGD_B1Cp, tgd[1]=TGD_B2ap
                       ///< BDS CNAV3:	tgd[2]=TGD_B2bI

  f64 ttms = 0;  ///< transmission time (s) within week
};

/** system Time offset message
 */
struct NAVP_EXPORT STO {
  NavMsgTypeEnum type = NavMsgTypeEnum::NONE;  ///< message type
  Sv Sat;                                      ///< satellite number
  utils::GTime tot;                            ///< reference epoch for time offset information
  utils::GTime ttm;                            ///< transmission time
  StoCodeEnum code = StoCodeEnum::NONE;        ///< system Time offset code;
  SbasIdEnum sid = SbasIdEnum::NONE;           ///< SBAS ID
  UtcIdEnum uid = UtcIdEnum::NONE;             ///< UTC ID

  f64 A0 = 0;  ///< (sec)
  f64 A1 = 0;  ///< (sec/sec)
  f64 A2 = 0;  ///< (sec/sec^2)

  f64 ttms = 0;  ///< transmission time (s) within week
};

/** EOP message
 */
struct NAVP_EXPORT EOP {
  NavMsgTypeEnum type = NavMsgTypeEnum::NONE;  ///< message type
  Sv Sat;                                      ///< satellite number
  utils::GTime teop;                           ///< reference epoch of EOP data
  utils::GTime ttm;                            ///< transmission time

  f64 xp = 0;    ///< pole offset (rad)
  f64 xpr = 0;   ///< pole offset rate (rad/day)
  f64 xprr = 0;  ///< pole offset rate rate (rad/day^2)
  f64 yp = 0;    ///< pole offset (rad)
  f64 ypr = 0;   ///< pole offset rate (rad/day)
  f64 yprr = 0;  ///< pole offset rate rate (rad/day^2)
  f64 dut1 = 0;  ///< ut1-utc or ut1-gpst (s)
  f64 dur = 0;   ///< delta ut1 rate (s/day)
  f64 durr = 0;  ///< delta ut1 rate rate (s/day^2)

  f64 ttms;  ///< transmission time (s) within week
};

/** ionosphere message
 */
struct NAVP_EXPORT ION {
  NavMsgTypeEnum type = NavMsgTypeEnum::NONE;  ///< message type
  Sv Sat;                                      ///< satellite number
  utils::GTime ttm;                            ///< transmission time
  i32 code = 0;                                ///< rgion code for QZS
  i32 flag = 0;                                ///< disturbance flags for GAL

  union {
    f64 vals[9] = {};
    struct {
      // Klobuchar model: GPS/QZS LNAV/CNVX and BDS D1D2
      f64 a0;
      f64 a1;
      f64 a2;
      f64 a3;
      f64 b0;
      f64 b1;
      f64 b2;
      f64 b3;
    };
    struct {
      // NEQUICK-G model: GAL IFNV
      f64 ai0;
      f64 ai1;
      f64 ai2;
    };
    struct {
      // BDGIM model: BDS CNVX
      f64 alpha1;
      f64 alpha2;
      f64 alpha3;
      f64 alpha4;
      f64 alpha5;
      f64 alpha6;
      f64 alpha7;
      f64 alpha8;
      f64 alpha9;
    };
  };
};

}  // namespace navp::sensors::gnss