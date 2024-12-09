#pragma once

#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace navp::sensors::gnss {

enum class NAVP_EXPORT ConstellationEnum : u8 {
  GPS,
  BDS,
  GAL,
  GLO,
  QZS,
  IRN,
  SBS,
  LEO,

  WAAS,
  EGNOS,
  MSAS,
  GAGAN,
  BDSBAS,
  KASS,
  SDCM,
  ASBAS,
  SPAN,
  AusNZ,
  GBAS,
  NSAS,
  ASAL,
  Mixed,

  IMS,
  COMB,
  NONE,
};

enum class NAVP_EXPORT TimeSystemEnum : u8 {
  NONE,      ///< NONE for unknown
  GPST,      ///< GPS Time
  GLONASST,  ///< GLONASS Time
  GST,       ///< Galileo System Time
  BDT,       ///< BeiDou Time
  QZSST,     ///< QZSS Time
  TAI,       ///< International Atomic Time
  UTC,       ///< Universal Coordinated Time
  UT1,       ///< Universal Time corrected for polar motion
  TT         ///< Terrestrial Time
};

enum class NAVP_EXPORT NavMsgTypeEnum : u8 {
  NONE,  ///< NONE for unknown
  LNAV,  ///< GPS/QZSS/NavIC Legacy Navigation Messages
  FDMA,  ///< GLONASS Legacy FDMA Navigation Message
  FNAV,  ///< Galileo Free Navigation Message
  INAV,  ///< Galileo Integrity Navigation Message
  IFNV,  ///< Galileo INAV or FNAV Navigation Message
  D1,    ///< BeiDou-2/3 MEO/IGSO Navigation Message
  D2,    ///< BeiDou-2/3 GEO Navigation Message
  D1D2,  ///< BeiDou-2/3 MEO/IGSO and GEO Navigation Message
  SBAS,  ///< SBAS Navigation Message
  CNAV,  ///< GPS/QZSS CNAV Navigation Message
  CNV1,  ///< BeiDou-3 CNAV-1 Navigation Message
  CNV2,  ///< GPS/QZSS CNAV-2 Navigation Message	  BeiDou-3 CNAV-2 Navigation Message
  CNV3,  ///< BeiDou-3 CNAV-3 Navigation Message
  CNVX   ///< GPS/QZSS CNAV or CNAV-2 Navigation Message  BeiDou-3 CNAV-1, CNAV-2 or CNAV-3 Navigation Message
};

enum class NAVP_EXPORT SvhEnum : i8 {
  SVH_OK,
  SVH_UNHEALTHY = -1  // implicitly used in rtcm
};

enum class NAVP_EXPORT MonthEnum : u8 { NONE, JAN, FEB, MAR, ARP, MAY, JUN, JUL, AUG, SEP, OCT, NOV, DEC };

enum class NAVP_EXPORT SatTypeEnum : u8 { NONE, GEO, IGSO, MEO };

enum class NAVP_EXPORT ObxFrameEnum : u8 { OTHER, ECEF, ECI, BCRS };

enum class NAVP_EXPORT StoCodeEnum : u8 {
  NONE,
  GPUT,
  GLUT,
  GLGP,
  GAUT,
  GAGP,
  GPGA = GAGP,  // From RINEX 3.04 the GPGA label is replaced by GAGP, while the value and sign for the Galileo minus
                // GPS time offset remains unchanged.
  GAGL,
  BDUT,
  BDGP,
  BDGL,
  BDGA,
  QZUT,
  QZGP,
  QZGL,
  QZGA,
  QZBD,
  IRUT,
  IRGP,
  IRGL,
  IRGA,
  IRBD,
  IRQZ,
  SBUT,
  SBGP,
  SBGL,
  SBGA,
  SBBD,
  SBQZ,
  SBIR
};

enum class NAVP_EXPORT SbasIdEnum : u8 { NONE, WAAS, EGNOS, MSAS, GAGAN, SDCM, BDSBAS, KASS, A_SBAS, SPAN };

enum class NAVP_EXPORT UtcIdEnum : u8 {
  NONE,
  UTC_USNO,
  UTC_SU,
  UTCGAL,
  UTC_NTSC,
  UTC_NICT,
  UTC_NPLI,
  UTCIRN,
  UTC_OP,
  UTC_NIST
};

enum class NAVP_EXPORT ObsCodeEnum : u16 {
  NONE,  ///< none or unknown
  L1C,   ///< L1C/A,G1C/A,E1C		(GPS,GLO,GAL,QZS,SBS)
  L1P,   ///< L1P,G1P    			(GPS,GLO)
  L1W,   ///< L1 Z-track 			(GPS)
  L1Y,   ///< L1Y        			(GPS)
  L1M,   ///< L1M        			(GPS)
  L1N,   ///< L1codeless 			(GPS)
  L1S,   ///< L1C(D)     			(GPS,QZS)
  L1L,   ///< L1C(P)     			(GPS,QZS)
  L1E,   ///< L1C/B      			(QZS)
  L1A,   ///< E1A        			(GAL)
  L1B,   ///< E1B        			(GAL)
  L1X,   ///< E1B+C,L1C(D+P)			(GAL,QZS)
  L1Z,   ///< E1A+B+C,L1-SAIF		(GAL,QZS)
  L1R,   ///< M (RMP antenna) (GPS) from rinex 4.01
  L1I,   ///< B1I        			(BDS)
  L1D,   ///< B1D        			(BDS)
  L1Q,   ///< B1Q        			(BDS)

  L2C,  ///< L2C/A,G1C/A			(GPS,GLO)
  L2D,  ///< L2 L1C/A-(P2-P1)		(GPS)
  L2S,  ///< L2C(M)     			(GPS,QZS)
  L2L,  ///< L2C(L)     			(GPS,QZS)
  L2X,  ///< L2C(M+L),B1-2I+Q		(GPS,QZS,BDS)
  L2P,  ///< L2P,G2P    			(GPS,GLO)
  L2W,  ///< L2 Z-track 			(GPS)
  L2Y,  ///< L2Y        			(GPS)
  L2M,  ///< L2M        			(GPS)
  L2N,  ///< L2codeless 			(GPS)
  L2R,  ///< M (RMP antenna) (GPS) from rinex 4.01
  L2I,  ///< B1-2I      			(BDS)
  L2Q,  ///< B1-2Q      			(BDS)

  L3I,  ///< G3I        			(GLO)
  L3Q,  ///< G3Q        			(GLO)
  L3X,  ///< G3I+Q      			(GLO)

  L4A,  ///< L1OCd					(GLO)
  L4B,  ///< L1OCp					(GLO)
  L4X,  ///< L1OCd+L1OCp			(GLO)

  L5I,  ///< L5/E5aI    			(GPS,GAL,QZS,SBS)
  L5Q,  ///< L5/E5aQ    			(GPS,GAL,QZS,SBS)
  L5X,  ///< L5/E5aI+Q  			(GPS,GAL,QZS,SBS)
  L5D,  ///< B2aD       			(BDS)
  L5P,  ///< B2aP       			(BDS)
  L5A,  ///< L5 A SPS				(IRN)
  L5B,  ///< L5 B RS(D)				(IRN)
  L5C,  ///< L5 C RS(P)				(IRN)
  L5Z,  ///< L5 B+C       			(IRN)

  L6A,  ///< E6A, L2OCd       		(GAL,GLO)
  L6B,  ///< E6B, L2OCp        		(GAL,GLO)
  L6C,  ///< E6C, L2OCd+L2OCp       (GAL,GLO)
  L6X,  ///< E6B+C,LEXS+L,B3I+Q 	(GAL,QZS,BDS)
  L6Z,  ///< E6A+B+C    			(GAL)
  L6S,  ///< L6S    				(QZS)
  L6L,  ///< L6L    				(QZS)
  L6I,  ///< B3I        			(BDS)
  L6Q,  ///< B3Q        			(BDS)
  L6E,  ///< L6E					(QZS)
  L6D,  ///< L6 Data       			(BDS)
  L6P,  ///< L6 Pilot       		(BDS)

  L7D,  ///< L7 Data       			(BDS)
  L7P,  ///< L7 Pilot       		(BDS)
  L7Z,  ///< L7 Data+Pilot       	(BDS)
  L7I,  ///< E5bI,B2aI  			(GAL,BDS)
  L7Q,  ///< E5bQ,B2aQ  			(GAL,BDS)
  L7X,  ///< E5bI+Q,B2aI+Q			(GAL,BDS)

  L8I,  ///< E5(a+b)I   			(GAL)
  L8Q,  ///< E5(a+b)Q   			(GAL)
  L8X,  ///< E5(a+b)I+Q 			(GAL)
  L8D,  ///< L8 Data       			(BDS)
  L8P,  ///< L8 Pilot       		(BDS)
  L8Z,  ///< L8 Data+Pilot       	(BDS)

  L9A,  ///< S9 A SPS				(IRN)
  L9B,  ///< S9 B RS(D)        		(IRN)
  L9C,  ///< S9 C RS(P)       		(IRN)
  L9X,  ///< S9 B+C       			(IRN)

  AUTO,
};

enum class NAVP_EXPORT ObsCode2Enum : u8 {
  NONE,
  P1,
  P2,
  C1,
  C2,
  C3,
  C4,
  C5,
  C6,
  C7,
  C8,
  L1,
  L2,
  L3,
  L4,
  L5,
  L6,
  L7,
  L8,
  LA
};

enum class NAVP_EXPORT TrigTypeEnum : u16 { COS, SIN };

enum class NAVP_EXPORT FreTypeEnum : u8 {
  FTYPE_NONE,
  /* Base carrier frequencies */
  F1,   //  1575.42  MHz: GPS L1, GAL E1,  BDS-3 B1C,  QZS L1,  SBS L1,
  F2,   //  1227.60  MHz: GPS L2, QZS L2,
  F5,   //  1176.45  MHz: GPS L5, GAL E5A, BDS-3 B2A, QZS L5, SBS L5
  F6,   //  1278.75  MHz: GAL E6, QZS L6,
  F7,   //  1207.14  MHz: GAL E5B, BDS-3 B2B, BDS-2 B2I
  F8,   //  1191.795 MHz: GAL E5
  G1,   // ~1602     MHz: GLO G1,
  G2,   // ~1246     MHz: GLO G2,
  G3,   //  1202.025 MHz: GLO G3,
  G1A,  //  1600.995 MHz  GLO G1A
  G2A,  //  1248.06  MHz: GLO G2A,
  B1,   //  1561.098 MHz: BDS-2/3 B1I,
  B3,   //  1268.52  MHz: BDS-2/3 B3I,
  I9,   //  2492.028 MHz: IRN S9
  // NUM_FTYPES,

  // alias
  L1 = F1,  // GPS/SBAS/QZS L1
  L2 = F2,  // GPS/QZS L2
  L5 = F5,  // GPS/SBAS/QZS L5

  E1 = F1,   // GAL E1
  E5A = F5,  // GAL E5a
  E5B = F7,  // GAL E5b
  E5 = F8,   // GAL E5(E5a+E5b)
  E6 = F6,   // GAL E6

  L6 = F6,  // QZS F6

  B1C = F1,   // BDS-3 B1c
  B1A = F1,   // BDS-3 B1a
  B2A = F5,   // BDS-3 B2a
  B2 = F7,    // BDS-2
  B2B = F7,   // BDS-3 B2b
  B2AB = F8,  // BDS-3 B2(B2a+B2b)
  B3A = B3,   // BDS-3 B3A
};

/// unused
enum class NAVP_EXPORT IonoModeEnum : u8 {
  OFF,                     ///< ionosphere option: correction off
  BROADCAST,               ///< ionosphere option: broadcast model
  SBAS,                    ///< ionosphere option: SBAS model
  IONO_FREE_LINEAR_COMBO,  ///< ionosphere option: L1/L2 or L1/L5 iono-free LC
  ESTIMATE,                ///< ionosphere option: estimation
  TOTAL_ELECTRON_CONTENT,  ///< ionosphere option: IONEX TEC model
  QZS,                     ///< ionosphere option: QZSS broadcast model
  LEX,                     ///< ionosphere option: QZSS LEX ionospehre
  STEC                     ///< ionosphere option: SLANT TEC model
};

/// unused
enum class NAVP_EXPORT IonoModelEnum : u8 { NONE, MEAS_OUT, BSPLINE, SPHERICAL_CAPS, SPHERICAL_HARMONICS, LOCAL };

/// unused
enum class NAVP_EXPORT RandomModelEnum : u8 {
  STANDARD,             /// Setting fixed error
  ELEVATION_DEPENDENT,  /// Elevation model
  SNR_DEPENDENT,        /// SNR model
  CUSTOM,               /// Custom Model(not support now)
};

/// unused
enum class NAVP_EXPORT TropModelEnum : u8 {
  STANDARD,  ///< Saastamoinen model
  SBAS,
  VMF3,
  GPT2,
  CSSR
};

/// unused
enum class NAVP_EXPORT IonoMapFnEnum : u8 {
  SLM,       ///< single layer model mapping function
  MSLM,      ///< modified single layer model mapping function
  MLM,       ///< multiple layer model mapping function
  KLOBUCHAR  ///< Klobuchar mapping function
};

enum class NAVP_EXPORT NavRecTypeEnum : u8 {
  NONE,  ///< NONE for unknown */
  EPH,   ///< Ephemerides data including orbit, clock, biases, accuracy and status parameters */
  STO,   ///< System Time and UTC proxy offset parameters */
  EOP,   ///< Earth Orientation Parameters */
  ION    ///< Global/Regional ionospheric model parameters */
};

}  // namespace navp::sensors::gnss