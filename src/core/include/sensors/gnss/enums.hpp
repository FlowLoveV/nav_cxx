#pragma once

#include "utils/types.hpp"
#include "utils/macro.hpp"

namespace navp::sensors::gnss {

enum class NAVP_EXPORT ConstellationEnum : u8 {
  NONE,
  GPS,
  GAL,
  GLO,
  QZS,
  SBS,
  BDS,
  LEO,
  SUPPORTED,
  IRN,
  IMS,
  COMB,

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

enum class NAVP_EXPORT UtcIdEnum : u8 { NONE, UTC_USNO, UTC_SU, UTCGAL, UTC_NTSC, UTC_NICT, UTC_NPLI, UTCIRN, UTC_OP, UTC_NIST };

enum class NAVP_EXPORT ObsCodeEnum : u16 {
  NONE = 0,  ///< none or unknown
  L1C = 1,   ///< L1C/A,G1C/A,E1C		(GPS,GLO,GAL,QZS,SBS)
  L1P = 2,   ///< L1P,G1P    			(GPS,GLO)
  L1W = 3,   ///< L1 Z-track 			(GPS)
  L1Y = 4,   ///< L1Y        			(GPS)
  L1M = 5,   ///< L1M        			(GPS)
  L1N = 6,   ///< L1codeless 			(GPS)
  L1S = 7,   ///< L1C(D)     			(GPS,QZS)
  L1L = 8,   ///< L1C(P)     			(GPS,QZS)
  L1E = 9,   ///< L1C/B      			(QZS)
  L1A = 10,  ///< E1A        			(GAL)
  L1B = 11,  ///< E1B        			(GAL)
  L1X = 12,  ///< E1B+C,L1C(D+P)			(GAL,QZS)
  L1Z = 13,  ///< E1A+B+C,L1-SAIF		(GAL,QZS)
  L2C = 14,  ///< L2C/A,G1C/A			(GPS,GLO)
  L2D = 15,  ///< L2 L1C/A-(P2-P1)		(GPS)
  L2S = 16,  ///< L2C(M)     			(GPS,QZS)
  L2L = 17,  ///< L2C(L)     			(GPS,QZS)
  L2X = 18,  ///< L2C(M+L),B1-2I+Q		(GPS,QZS,BDS)
  L2P = 19,  ///< L2P,G2P    			(GPS,GLO)
  L2W = 20,  ///< L2 Z-track 			(GPS)
  L2Y = 21,  ///< L2Y        			(GPS)
  L2M = 22,  ///< L2M        			(GPS)
  L2N = 23,  ///< L2codeless 			(GPS)
  L5I = 24,  ///< L5/E5aI    			(GPS,GAL,QZS,SBS)
  L5Q = 25,  ///< L5/E5aQ    			(GPS,GAL,QZS,SBS)
  L5X = 26,  ///< L5/E5aI+Q  			(GPS,GAL,QZS,SBS)
  L7I = 27,  ///< E5bI,B2aI  			(GAL,BDS)
  L7Q = 28,  ///< E5bQ,B2aQ  			(GAL,BDS)
  L7X = 29,  ///< E5bI+Q,B2aI+Q			(GAL,BDS)
  L6A = 30,  ///< E6A, L2OCd       		(GAL,GLO)
  L6B = 31,  ///< E6B, L2OCp        		(GAL,GLO)
  L6C = 32,  ///< E6C, L2OCd+L2OCp       (GAL,GLO)
  L6X = 33,  ///< E6B+C,LEXS+L,B3I+Q 	(GAL,QZS,BDS)
  L6Z = 34,  ///< E6A+B+C    			(GAL)
  L6S = 35,  ///< L6S    				(QZS)
  L6L = 36,  ///< L6L    				(QZS)
  L8I = 37,  ///< E5(a+b)I   			(GAL)
  L8Q = 38,  ///< E5(a+b)Q   			(GAL)
  L8X = 39,  ///< E5(a+b)I+Q 			(GAL)
  L2I = 40,  ///< B1-2I      			(BDS)
  L2Q = 41,  ///< B1-2Q      			(BDS)
  L6I = 42,  ///< B3I        			(BDS)
  L6Q = 43,  ///< B3Q        			(BDS)
  L3I = 44,  ///< G3I        			(GLO)
  L3Q = 45,  ///< G3Q        			(GLO)
  L3X = 46,  ///< G3I+Q      			(GLO)
  L1I = 47,  ///< B1I        			(BDS)
  L1Q = 48,  ///< B1Q        			(BDS)
  L4A = 49,  ///< L1OCd					(GLO)
  L4B = 50,  ///< L1OCp					(GLO)
  L4X = 51,  ///< L1OCd+L1OCp			(GLO)
  L6E = 52,  ///< L6E					(QZS)
  L1D = 53,  ///< B1D        			(BDS)
  L5D = 54,  ///< B2aD       			(BDS)
  L5P = 55,  ///< B2aP       			(BDS)
  L9A = 57,  ///< S9 A SPS				(IRN)
  L9B = 58,  ///< S9 B RS(D)        		(IRN)
  L9C = 59,  ///< S9 C RS(P)       		(IRN)
  L9X = 60,  ///< S9 B+C       			(IRN)
  L5A = 61,  ///< L5 A SPS				(IRN)
  L5B = 62,  ///< L5 B RS(D)				(IRN)
  L5C = 63,  ///< L5 C RS(P)				(IRN)
  L5Z = 64,  ///< L5 B+C       			(IRN)
  L6D = 65,  ///< L6 Data       			(BDS)
  L6P = 66,  ///< L6 Pilot       		(BDS)
  L7D = 67,  ///< L7 Data       			(BDS)
  L7P = 68,  ///< L7 Pilot       		(BDS)
  L7Z = 69,  ///< L7 Data+Pilot       	(BDS)
  L8D = 70,  ///< L8 Data       			(BDS)
  L8P = 71,  ///< L8 Pilot       		(BDS)
  L8Z = 72,  ///< L8 Data+Pilot       	(BDS)
  AUTO = 200,
};

enum class NAVP_EXPORT ObsCode2Enum : u8 { NONE, P1, P2, C1, C2, C3, C4, C5, C6, C7, C8, L1, L2, L3, L4, L5, L6, L7, L8, LA };

enum class NAVP_EXPORT TrigTypeEnum : u16 { COS, SIN };

enum class NAVP_EXPORT FreTypeEnum : u8 {
  FTYPE_NONE,
  /* Base carrier frequencies */
  F1 = 1,   //  1575.42  MHz: GPS L1, GAL E1,  BDS B1C,  QZS L1,  SBS L1,
  F2 = 2,   //  1227.60  MHz: GPS L2, QZS L2,
  F5 = 5,   //  1176.45  MHz: GPS L5, GAL E5A, BDS B2A, QZS L5, SBS L5
  F6 = 6,   //  1278.75  MHz: GAL E6, QZS L6,
  F7 = 7,   //  1207.14  MHz: GAL E5B, BDS B2B
  F8 = 8,   //  1191.795 MHz: GAL E5, BDS B2,
  G1 = 11,  // ~1602     MHz: GLO G1,
  G2 = 12,  // ~1246     MHz: GLO G2,
  G3 = 13,  //  1202.025 MHz: GLO G3,
  G4 = 14,  //  1600.995 MHz  GLO G1A
  G6 = 16,  //  1248.08  MHz: GLO G2A,
  B1 = 21,  //  1561.098 MHz: BDS B1I,
  B3 = 23,  //  1268.52  MHz: BDS B3I,
  I9 = 39,  //  2492.028 MHz: IRN S9
  NUM_FTYPES,
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
enum class NAVP_EXPORT NoiseModelEnum : u8 {
  EQUAL_WEIGHT,         /// Equal Weight Model
  ELEVATION_DEPENDENT,  /// Elevation model
  SNR_DEPENDENT,        /// SNR model
  CUSTOM,               /// Custom Model(not support now)
};

/// unused
enum class NAVP_EXPORT TropModelEnum : u8 { STANDARD, SBAS, VMF3, GPT2, CSSR };

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