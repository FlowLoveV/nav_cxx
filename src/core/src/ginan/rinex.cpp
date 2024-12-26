#include "ginan/common.hpp"
#include "io/rinex/rinex_reader.hpp"
#include "sensors/gnss/constants.hpp"
#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/observation.hpp"
#include "sensors/gnss/sv.hpp"

using namespace navp::sensors::gnss;
using namespace navp::io::rinex;
using namespace navp::utils;
using namespace navp::ginan;

using navp::f64;
using navp::i16;
using navp::i32;
using navp::u16;
using navp::u8;

#define MAXPOSHEAD 1024  ///< max head line position
#define MINFREQ_GLO -7   ///< min frequency number glonass
#define MAXFREQ_GLO 13   ///< max frequency number glonass

inline auto constexpr PI = 3.141592653589793238462643383279502884197169399375105820974;
inline auto constexpr D2R = PI / 180.0;
inline auto constexpr R2D = 180.0 / PI;
inline auto constexpr AS2R = R2D / 3600.0;

enum class EphemerisType : unsigned {
  NONE,  ///< NONE for unknown
  EPH,   ///< GPS/QZS LNAV, GAL IFNV, BDS D1D2 Ephemeris
  GEPH,  ///< GLO Ephemeris
  SEPH,  ///< SBAS Ephemeris
  CEPH,  ///< GPS/QZS/BDS CNVX Ephemeris
  STO,   ///< STO message
  EOP,   ///< EOP message
  ION
};

/** Default navigation massage type for RINEX 3 and 2
 */
std::map<ConstellationEnum, NavMsgTypeEnum> NavMsgTypeMap = {
    {ConstellationEnum::GPS, NavMsgTypeEnum::LNAV}, {ConstellationEnum::GLO, NavMsgTypeEnum::FDMA},
    {ConstellationEnum::GAL, NavMsgTypeEnum::IFNV}, {ConstellationEnum::BDS, NavMsgTypeEnum::D1D2},
    {ConstellationEnum::QZS, NavMsgTypeEnum::LNAV}, {ConstellationEnum::IRN, NavMsgTypeEnum::LNAV},
    {ConstellationEnum::SBS, NavMsgTypeEnum::SBAS}};

/** Set string without tail space
 */
void setstr(char* dst, const char* src, i32 n) {
  char* p = dst;
  const char* q = src;

  while (*q && q < src + n) *p++ = *q++;

  *p-- = '\0';

  while (p >= dst && *p == ' ') *p-- = '\0';
}
namespace navp::io::rinex {

/** Decode obs header
 */
void decodeObsH(std::istream& inputStream, string& line, f64 ver, TimeSystemEnum& tsys,
                std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes, char* glo_fcn, f64* glo_cpbias,
                RinexStation& rnxRec) {
  f64 del[3];
  i32 prn;
  i32 fcn;
  const char* p;
  char* buff = &line[0];
  char* label = buff + 60;

  //	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << ": ver=" << ver;

  if (strstr(label, "MARKER NAME")) {
    if (rnxRec.id.empty()) {
      rnxRec.id.assign(buff, 4);
    }
  } else if (strstr(label, "MARKER NUMBER")) {
    rnxRec.marker.assign(buff, 20);

  }
  //     else if (strstr(label,"MARKER TYPE"         )) ; // ver.3
  //     else if (strstr(label,"OBSERVER / AGENCY"   )) ;
  else if (strstr(label, "REC # / TYPE / VERS")) {
    rnxRec.recSerial.assign(buff, 20);
    rnxRec.recType.assign(buff + 20, 20);
    rnxRec.recFWVersion.assign(buff + 40, 20);

  } else if (strstr(label, "ANT # / TYPE")) {
    rnxRec.antSerial.assign(buff, 20);
    rnxRec.antDesc.assign(buff + 20, 20);

  } else if (strstr(label, "APPROX POSITION XYZ")) {
    for (i32 i = 0, j = 0; i < 3; i++, j += 14) rnxRec.pos[i] = str2num(buff, j, 14);

  } else if (strstr(label, "ANTENNA: DELTA H/E/N")) {
    for (i32 i = 0, j = 0; i < 3; i++, j += 14) del[i] = str2num(buff, j, 14);

    rnxRec.del[2] = del[0];  // h
    rnxRec.del[0] = del[1];  // e
    rnxRec.del[1] = del[2];  // n

  }
  //     else if (strstr(label,"ANTENNA: DELTA X/Y/Z")) ; // opt ver.3
  //     else if (strstr(label,"ANTENNA: PHASECENTER")) ; // opt ver.3
  //     else if (strstr(label,"ANTENNA: B.SIGHT XYZ")) ; // opt ver.3
  //     else if (strstr(label,"ANTENNA: ZERODIR AZI")) ; // opt ver.3
  //     else if (strstr(label,"ANTENNA: ZERODIR XYZ")) ; // opt ver.3
  //     else if (strstr(label,"CENTER OF MASS: XYZ" )) ; // opt ver.3
  else if (strstr(label, "SYS / # / OBS TYPES")) {
    // ver.3
    // get system from code letter
    char code[] = "x00";
    code[0] = buff[0];

    auto sat_res = Sv::from_str(code);
    if (sat_res.is_err()) {
      return;
    }
    auto sv = sat_res.unwrap_unchecked();

    i32 n = (i32)str2num(buff, 3, 3);

    for (i32 j = 0, k = 7; j < n; j++, k += 4) {
      if (k > 58) {
        // more on the next line

        if (!std::getline(inputStream, line)) break;

        buff = &line[0];
        k = 7;
      }

      CodeType codeType;
      codeType.type = buff[k];

      char code[] = "Lxx";
      code[1] = buff[k + 1];
      code[2] = buff[k + 2];
      if ((sv.system() == ConstellationEnum::BDS) && (code[1] == '1') && (ver == 3.02)) {
        // change beidou B1 code: 3.02 draft -> 3.02
        code[1] = '2';
      }
      try {
        codeType.code = magic_enum::enum_cast<ObsCodeEnum>(code).value();
      } catch (...) {
        // BOOST_LOG_TRIVIAL(debug)
        // << "invalid obs code: " << code;
      }

      sysCodeTypes[sv.system()][j] = codeType;
    }

    // if unknown code in ver.3, set default code
    // 		for (auto& codeType : sysCodeTypes[sv.system()])
    // 		{
    //             if (tobs[i][j][2])
    // 				continue;
    //
    //             if (!(p = strchr(frqcodes, tobs[i][j][1])))
    // 				continue;
    //
    // default codes for unknown code
    //     			const char *defcodes[] =
    //    			{
    //         			"CWX   ",   // GPS: L125___
    //         			"CC    ",   // GLO: L12____
    //         			"X XXXX",   // GAL: L1_5678
    //         			"CXXX  ",   // QZS: L1256__
    //         			"C X   ",   // SBS: L1_5___
    //         			"X  XX "	// BDS: L1__67_
    //     			};
    //             	tobs[i][j][2] = defcodes[i][(i32)(p - frqcodes)];
    //
    //             	BOOST_LOG_TRIVIAL(debug)
    // 				<< "set default for unknown code: sys=" << buff[0]
    // 				<< " code=" << tobs[i][j];
    //         }
  }
  //     else if (strstr(label,"WAVELENGTH FACT L1/2")) ; // opt ver.2
  else if (strstr(label, "# / TYPES OF OBSERV")) {
    // ver.2

    i32 n = (i32)str2num(buff, 0, 6);

    for (i32 i = 0, j = 10; i < n; i++, j += 6) {
      if (j > 58) {
        // go onto new line
        if (!std::getline(inputStream, line)) break;

        buff = (char*)line.c_str();

        j = 10;
      }

      if (ver <= 2.99) {
        char obsCode2str[3] = {};
        setstr(obsCode2str, buff + j, 2);

        // save the type char before cleaning the string
        char typeChar = obsCode2str[0];

        for (ConstellationEnum sys : magic_enum::enum_values<ConstellationEnum>()) {
          // auto& recOpts = navp::ginan::acsConfig.getRecOpts(rnxRec.id, {std::string(magic_enum::enum_name(sys))});

          std::map<ObsCode2Enum, ObsCodeEnum>* conversionMap_ptr;
          if (typeChar != 'C' && typeChar != 'P') {
            obsCode2str[0] = 'L';
            // conversionMap_ptr = &recOpts.rinex23Conv.phasConv;  // ie use Phase Conversions for phase, doppler etc.
          } else {
            // conversionMap_ptr = &recOpts.rinex23Conv.codeConv;
          }

          auto& conversionMap = *conversionMap_ptr;

          CodeType codeType;

          try {
            ObsCode2Enum obsCode2 = magic_enum::enum_cast<ObsCode2Enum>(obsCode2str).value();
            ObsCodeEnum obsCode = conversionMap[obsCode2];
            codeType.code = obsCode;
            codeType.type = typeChar;
          } catch (...) {
            // BOOST_LOG_TRIVIAL(warning) << "Warning: Unknown code in rinex file: " << obsCode2str;
          }

          sysCodeTypes[sys][i] = codeType;
        }
      }
    }
    //*tobs[0][nt]='\0';
  }
  //     else if (strstr(label, "SIGNAL STRENGTH UNIT")) ; // opt ver.3
  //     else if (strstr(label, "INTERVAL"            )) ; // opt
  else if (strstr(label, "TIME OF FIRST OBS")) {
    if (!strncmp(buff + 48, "GPS", 3))
      tsys = TimeSystemEnum::GPST;
    else if (!strncmp(buff + 48, "GLO", 3))
      tsys = TimeSystemEnum::UTC;
    else if (!strncmp(buff + 48, "GAL", 3))
      tsys = TimeSystemEnum::GST;
    else if (!strncmp(buff + 48, "QZS", 3))
      tsys = TimeSystemEnum::QZSST;  // ver.3.02
    else if (!strncmp(buff + 48, "BDT", 3))
      tsys = TimeSystemEnum::BDT;  // ver.3.02
  }
  //     else if (strstr(label, "TIME OF LAST OBS"    )) ; // opt
  //     else if (strstr(label, "RCV CLOCK OFFS APPL" )) ; // opt
  //     else if (strstr(label, "SYS / DCBS APPLIED"  )) ; // opt ver.3
  //     else if (strstr(label, "SYS / PCVS APPLIED"  )) ; // opt ver.3
  //     else if (strstr(label, "SYS / SCALE FACTOR"  )) ; // opt ver.3
  //     else if (strstr(label, "SYS / PHASE SHIFTS"  )) ; // ver.3.01
  else if (strstr(label, "GLONASS SLOT / FRQ #")) {
    // ver.3.02
    p = buff + 4;
    for (i32 i = 0; i < 8; i++, p += 8) {
      if (sscanf(p, "R%2d %2d", &prn, &fcn) < 2) continue;

      if (1 <= prn && prn <= 27) {
        glo_fcn[prn - 1] = fcn + 8;
      }
    }
  } else if (strstr(label, "GLONASS COD/PHS/BIS")) {
    // ver.3.02
    p = buff;
    for (i32 i = 0; i < 4; i++, p += 13) {
      if (strncmp(p + 1, "C1C", 3))
        glo_cpbias[0] = str2num(p, 5, 8);
      else if (strncmp(p + 1, "C1P", 3))
        glo_cpbias[1] = str2num(p, 5, 8);
      else if (strncmp(p + 1, "C2C", 3))
        glo_cpbias[2] = str2num(p, 5, 8);
      else if (strncmp(p + 1, "C2P", 3))
        glo_cpbias[3] = str2num(p, 5, 8);
    }
  }
  // else if (strstr(label, "LEAP SECONDS")) {
  //   // This would be GPS-UTC, and NOT optional as of RINEX 4
  //   nav.leaps = (i32)str2num(buff, 0, 6);
  // }
  //     else if (strstr(label, "# OF SALTELLITES"    )) ; // opt
  //     else if (strstr(label, "PRN / # OF OBS"      )) ; // opt
}

/** Decode nav header
 */
void decodeNavH(string& line,           ///< Line to decode
                ConstellationEnum sys,  ///< GNSS system
                Navigation& nav)        ///< Navigation data
{
  char* buff = &line[0];
  char* label = buff + 60;

  // 	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

  if (strstr(label, "ION ALPHA")) {
    // opt ver.2
    NavMsgTypeEnum type = NavMsgTypeMap[sys];
    GTime time = {};

    ION& ionEntry = nav.ionMap[sys][type][time];

    ionEntry.type = type;
    ionEntry.sv.system() = sys;
    ionEntry.ttm = time;

    for (i32 i = 0, j = 2; i < 4; i++, j += 12) ionEntry.vals[i] = str2num(buff, j, 12);
  } else if (strstr(label, "ION BETA")) {
    // opt ver.2
    NavMsgTypeEnum type = NavMsgTypeMap[sys];
    GTime time = {};

    ION& ionEntry = nav.ionMap[sys][type][time];

    ionEntry.type = type;
    ionEntry.sv.system() = sys;
    ionEntry.ttm = time;

    for (i32 i = 0, j = 2; i < 4; i++, j += 12) ionEntry.vals[i + 4] = str2num(buff, j, 12);
  } else if (strstr(label, "DELTA-UTC: A0,A1,T,W")) {
    // opt ver.2
    NavMsgTypeEnum type = NavMsgTypeMap[sys];
    StoCodeEnum code = StoCodeEnum::NONE;
    switch (sys) {
      case ConstellationEnum::GPS:
        code = StoCodeEnum::GPUT;
        break;
      case ConstellationEnum::QZS:
        code = StoCodeEnum::QZUT;
        break;
      case ConstellationEnum::GAL:
        code = StoCodeEnum::GAUT;
        break;
      default:
        break;
    }

    GTow tow = str2num(buff, 31, 9);
    GWeek week = (i32)str2num(buff, 40, 9);
    GTime time(week, tow);

    STO& stoEntry = nav.stoMap[code][type][time];

    stoEntry.type = type;
    stoEntry.sv.system() = sys;
    stoEntry.tot = time;
    stoEntry.code = code;

    stoEntry.A0 = str2num(buff, 3, 19);
    stoEntry.A1 = str2num(buff, 22, 19);
    stoEntry.A2 = 0;
  } else if (strstr(label, "IONOSPHERIC CORR")) {
    // opt ver.3
    char sysStr[4] = "";
    strncpy(sysStr, buff, 3);
    sys = magic_enum::enum_cast<ConstellationEnum>(sysStr).value();
    NavMsgTypeEnum type = NavMsgTypeMap[sys];
    GTime time = {};

    ION& ionEntry = nav.ionMap[sys][type][time];

    ionEntry.type = type;
    ionEntry.sv.system() = sys;
    ionEntry.sv.prn = str2num(buff, 55, 3);
    ionEntry.ttm = time;

    if (buff[3] == 'A' || buff[3] == ' ') {
      for (i32 i = 0, j = 5; i < 4; i++, j += 12) ionEntry.vals[i] = str2num(buff, j, 12);
    } else if (buff[3] == 'B') {
      for (i32 i = 0, j = 5; i < 4; i++, j += 12) ionEntry.vals[i + 4] = str2num(buff, j, 12);
    }
  } else if (strstr(label, "TIME SYSTEM CORR")) {
    // opt ver.3
    char codeStr[5] = "";
    strncpy(codeStr, buff, 4);
    StoCodeEnum code = magic_enum::enum_cast<StoCodeEnum>(codeStr).value();

    Sv sv;

    switch (code) {
      case StoCodeEnum::GPUT:
        sv.system() = ConstellationEnum::GPS;
        break;
      case StoCodeEnum::GLUT:
        sv.system() = ConstellationEnum::GLO;
        break;
      case StoCodeEnum::GAUT:
        sv.system() = ConstellationEnum::GAL;
        break;
      case StoCodeEnum::BDUT:
        sv.system() = ConstellationEnum::BDS;
        break;
      case StoCodeEnum::QZUT:
        sv.system() = ConstellationEnum::QZS;
        break;
      case StoCodeEnum::SBUT:
        sv.system() = ConstellationEnum::SBS;
        break;
      case StoCodeEnum::GAGP:
        sv.system() = ConstellationEnum::GAL;
        break;
      case StoCodeEnum::QZGP:
        sv.system() = ConstellationEnum::QZS;
        break;
      default:
        break;
    }
    // UTC ID skipped

    NavMsgTypeEnum type = NavMsgTypeMap[sv.system()];

    f64 sec = str2num(buff, 38, 7);
    f64 week = str2num(buff, 45, 5);
    GTime time = {};
    if (sv.system() != ConstellationEnum::BDS) {
      time = GTime(GWeek(week), GTow(sec));
    } else {
      time = GTime(BWeek(week), BTow(sec));
    }

    STO& stoEntry = nav.stoMap[code][type][time];

    stoEntry.type = type;
    stoEntry.sv = sv;
    stoEntry.tot = time;
    stoEntry.ttm = time;
    stoEntry.code = code;

    stoEntry.A0 = str2num(buff, 5, 17);
    stoEntry.A1 = str2num(buff, 22, 16);
    stoEntry.A2 = 0.0;
  }
  // else if (strstr(label, "LEAP SECONDS")) {
  //   // opt
  //   nav.leaps = (i32)str2num(buff, 0, 6);
  // }
}
/** Decode gnav header
 */
void decodeGnavH(string& line, Navigation& nav) {
  char* buff = &line[0];
  char* label = buff + 60;

  // 	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

  if (strstr(label, "CORR TO SYTEM TIME"))
    ;  // opt
  // else if (strstr(label, "LEAP SECONDS")) {
  //   // opt
  //   nav.leaps = (i32)str2num(buff, 0, 6);
  // }
}

/** Decode geo nav header
 */
void decodeHnavH(string& line, Navigation& nav) {
  char* buff = &line[0];
  char* label = buff + 60;

  // 	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

  if (strstr(label, "CORR TO SYTEM TIME"))
    ;  // opt
  else if (strstr(label, "D-UTC A0,A1,T,W,S,U"))
    ;  // opt
  // else if (strstr(label, "LEAP SECONDS")) {
  //   // opt
  //   nav.leaps = (i32)str2num(buff, 0, 6);
  // }
}

/** Read rinex header
 */
i32 readRnxH(std::istream& inputStream, f64& ver, char& type, ConstellationEnum& sys, TimeSystemEnum& tsys,
             std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes, Navigation& nav, RinexStation& rnxRec,
             char* glo_fcn, f64* glo_cpbias) {
  string line;
  i32 i = 0;
  i32 block = 0;

  // 	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

  ver = 2.10;
  type = ' ';
  sys = ConstellationEnum::GPS;
  tsys = TimeSystemEnum::GPST;

  char sysChar = '\0';
  i32 typeOffset = 20;
  i32 sysCharOffset = 40;

  while (std::getline(inputStream, line)) {
    char* buff = &line[0];
    char* label = buff + 60;

    if (line.length() <= 60) {
      continue;
    } else if (strstr(label, "RINEX VERSION / TYPE")) {
      ver = str2num(buff, 0, 9);

      type = buff[typeOffset];

      sysChar = buff[sysCharOffset];

      // possible error in generation by one manufacturer. This hack gets around it
      if (ver == 3.04 && type == ' ') {
        typeOffset += 1;
        sysCharOffset += 2;
        type = buff[typeOffset];

        sysChar = buff[sysCharOffset];
      }

      // satellite system
      switch (sysChar) {
        case ' ':
        case 'G':
          sys = ConstellationEnum::GPS;
          tsys = TimeSystemEnum::GPST;
          break;
        case 'R':
          sys = ConstellationEnum::GLO;
          tsys = TimeSystemEnum::UTC;
          break;
        case 'E':
          sys = ConstellationEnum::GAL;
          tsys = TimeSystemEnum::GST;
          break;  // v.2.12
        case 'S':
          sys = ConstellationEnum::SBS;
          tsys = TimeSystemEnum::GPST;
          break;
        case 'J':
          sys = ConstellationEnum::QZS;
          tsys = TimeSystemEnum::QZSST;
          break;  // v.3.02
        case 'C':
          sys = ConstellationEnum::BDS;
          tsys = TimeSystemEnum::BDT;
          break;  // v.2.12
        case 'M':
          sys = ConstellationEnum::Mixed;
          tsys = TimeSystemEnum::GPST;
          break;  // mixed
        default:
          break;
      }
      continue;
    } else if (strstr(label, "PGM / RUN BY / DATE"))
      continue;
    else if (strstr(label, "COMMENT")) {
      // read cnes wl satellite fractional bias
      if (strstr(buff, "WIDELANE SATELLITE FRACTIONAL BIASES") ||
          strstr(buff, "WIDELANE SATELLITE FRACTIONNAL BIASES")) {
        block = 1;
      }
      if (strstr(buff, "->")) {
        // may be a conversion line, test

        char sysChar;
        char r3[4] = {};
        char r2[3] = {};
        char comment[81];
        i32 num = sscanf(buff, " %c %3c -> %2c %80s", &sysChar, r3, r2, comment);

        if (num == 4 && (string)comment == "COMMENT") {
          try {
            ConstellationEnum sys = Constellation::form_str(&sysChar).unwrap().id;

            char code = r3[0];
            r3[0] = 'L';

            r2[0] = std::toupper(r2[0]);
            r3[1] = std::toupper(r3[1]);
            r3[2] = std::toupper(r3[2]);
            r3[3] = std::toupper(r3[3]);

            auto obs2 = magic_enum::enum_cast<ObsCode2Enum>(r2).value();
            auto obs3 = magic_enum::enum_cast<ObsCodeEnum>(r3).value();

            // auto& recOpts = navp::ginan::acsConfig.getRecOpts(rnxRec.id, {std::string(magic_enum::enum_name(sys))});

            // auto& codeMap = recOpts.rinex23Conv.codeConv;
            // auto& phasMap = recOpts.rinex23Conv.phasConv;

            if (r2[0] == 'C' || r2[0] == 'P') {
              // codeMap[obs2] = obs3;
            } else {
              // phasMap[obs2] = obs3;
            }
          } catch (...) {
          }
        }
      } else if (block) {
        // ignore reported widelane biases

        // f64 bias;
        // Sv sv;
        //
        // // cnes/cls grg clock
        // if	( !strncmp(buff, "WL", 2)
        // 	&&(sv = Sv(buff + 3), sv)
        // 	&& sscanf(buff+40, "%lf", &bias) == 1)
        // {
        // 	nav.satNavMap[sv].wlbias = bias;
        // }
        // // cnes ppp-wizard clock
        // else if ((sv = Sv(buff + 1), sv)
        // 		&&sscanf(buff+6, "%lf", &bias) == 1)
        // {
        // 	nav.satNavMap[sv].wlbias = bias;
        // }
      }
      continue;
    }
    // file type
    switch (type) {
      case 'O':
        decodeObsH(inputStream, line, ver, tsys, sysCodeTypes, glo_fcn, glo_cpbias, rnxRec);
        break;
      case 'N':
        decodeNavH(line, sys, nav);
        break;  // GPS (ver.2) or mixed (ver.3)
      case 'G':
        decodeGnavH(line, nav);
        break;
      case 'H':
        decodeHnavH(line, nav);
        break;
      case 'J':
        decodeNavH(line, ConstellationEnum::QZS, nav);
        break;   // extension
      case 'E':  // fallthrough
      case 'L':
        decodeNavH(line, ConstellationEnum::GAL, nav);
        break;  // extension
      default:
        break;
    }
    if (strstr(label, "END OF HEADER")) return 1;

    if (++i >= MAXPOSHEAD && type == ' ') {
      break;  // no rinex file
    }
  }
  return 0;
}
/** Decode obs epoch
 */
i32 decodeObsEpoch(std::istream& inputStream, string& line, f64 ver, TimeSystemEnum tsys, GTime& time, i32& flag,
                   std::vector<Sv>& sats) {
  i32 n = 0;
  char* buff = &line[0];

  // 	BOOST_LOG_TRIVIAL(debug)	<< __FUNCTION__ << ": ver=" << ver;

  if (ver <= 2.99) {
    // ver.2
    n = (i32)str2num(buff, 29, 3);
    if (n <= 0) return 0;

    // epoch flag: 3:new site,4:header info,5:external event
    flag = (i32)str2num(buff, 28, 1);

    if (flag >= 3 && flag <= 5) {
      return n;
    }

    bool error = str2time(buff, 0, 26, time, tsys);
    if (error) {
      // BOOST_LOG_TRIVIAL(debug)
      // << "rinex obs invalid epoch: epoch=" << buff;

      return 0;
    }

    for (i32 i = 0, j = 32; i < n; i++, j += 3) {
      if (j >= 68) {
        // more on the next line
        if (!std::getline(inputStream, line)) break;

        buff = &line[0];

        j = 32;
      }

      char id[4] = {};
      strncpy(id, buff + j, 3);
      sats.emplace_back(Sv::from_str(id).unwrap());
    }
  } else {
    // ver.3
    n = (i32)str2num(buff, 32, 3);
    if (n <= 0) {
      return 0;
    }

    flag = (i32)str2num(buff, 31, 1);

    if (flag >= 3 && flag <= 5) return n;

    if (buff[0] != '>' || str2time(buff, 1, 28, time, tsys)) {
      // BOOST_LOG_TRIVIAL(debug)
      // << "rinex obs invalid epoch: epoch=" << buff;
      return 0;
    }
  }

  //     BOOST_LOG_TRIVIAL(debug)
  // 	<< "__FUNCTION__: time=" << time.to_string(3)
  // 	<< " flag=" << flag;

  return n;
}

/** Decode obs data
 */
i32 decodeObsData(std::istream& inputStream, string& line, f64 ver,
                  std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes, GObs& obs, Sv& v2Sv) {
  char satid[8] = "";
  i32 stat = 1;
  char* buff = &line[0];

  if (ver > 2.99) {
    // ver.3
    strncpy(satid, buff, 3);
    obs.sv = Sv::from_str(satid).unwrap();
  } else {
    obs.sv = v2Sv;
  }

  if (!obs.sv) {
    stat = 0;
  }

  auto& codeTypes = sysCodeTypes[obs.sv.system()];

  i32 j;
  if (ver <= 2.99)
    j = 0;
  else
    j = 3;

  if (!stat) return 0;

  for (auto& [index, codeType] : codeTypes) {
    if (ver <= 2.99 && j >= 80) {
      // ver.2
      if (!std::getline(inputStream, line)) break;
      buff = &line[0];
      j = 0;
    }

    FreTypeEnum ft = Constants::code_to_freq_enum(obs.sv.system(), codeType.code);

    RawSig* rawSig = nullptr;
    auto& sigList = obs.sigs_list[ft];

    for (auto& sig : sigList) {
      if (sig.code == codeType.code) {
        rawSig = &sig;
        break;
      }
    }

    if (rawSig == nullptr) {
      RawSig raw;
      raw.code = codeType.code;

      sigList.emplace_back(Sig{raw});
      rawSig = &sigList.back();
    }

    f64 val = str2num(buff, j, 14);
    f64 lli = str2num(buff, j + 14, 1);
    lli = (u8)lli & 0x03;

    RawSig& sig = *rawSig;
    if (val) switch (codeType.type) {
        case 'P':  // fallthrough
        case 'C':
          sig.pseudorange = val;
          break;
        case 'L':
          sig.carrier = val;
          sig.lli = static_cast<decltype(Sig::lli)>(lli);
          break;
        case 'D':
          sig.doppler = val;
          break;
        case 'S':
          sig.snr = static_cast<decltype(Sig::snr)>(val);
          break;
        default:
          break;
      }
    j += 16;
  }

  return 1;
}

/// read observation at next epoch
int readNextRnxObsB(std::istream& inputStream, double ver, TimeSystemEnum tsys,
                    std::map<ConstellationEnum, std::map<int, CodeType>>& sysCodeTypes, int& flag, ObsList& obsList) {
  GTime time = {};
  int i = 0;
  int nSats = 0;  // cant replace with sats.size()
  std::vector<Sv> sats;

  // read record
  string line;
  std::streampos pos;
  while (pos = inputStream.tellg(), std::getline(inputStream, line)) {
    // decode obs epoch
    if (i == 0) {
      nSats = decodeObsEpoch(inputStream, line, ver, tsys, time, flag, sats);
      if (nSats <= 0) {
        continue;
      }
    } else if (line[0] == '>') {
      inputStream.seekg(pos);
      return obsList.size();
    } else if (flag <= 2 || flag == 6) {
      GObs rawObs = {};
      rawObs.time = time;
      // decode obs data
      bool pass = decodeObsData(inputStream, line, ver, sysCodeTypes, rawObs, sats[i - 1]);
      rawObs.check_vaild();
      if (pass) {
        // save obs data
        obsList.emplace_back(std::make_shared<GObs>(std::move(rawObs)));
      }
    }
    i++;
    if (i > nSats) return obsList.size();
  }
  return -1;
}

/** Read rinex obs
 */
i32 readRnxObs(std::istream& inputStream, f64 ver, TimeSystemEnum tsys,
               std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes, ObsList& obsList,
               RinexStation& rnxRec) {
  i32 flag = 0;
  i32 stat = 0;

  // read rinex obs data body
  i32 n = readNextRnxObsB(inputStream, ver, tsys, sysCodeTypes, flag, obsList);

  if (n >= 0) stat = 1;

  return stat;
}

/** Decode ephemeris
 */
i32 decodeEph(f64 ver, Sv sv, GTime toc, std::vector<f64>& data, Eph& eph) {
  // 	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << ": ver=" << ver << " sat=" << sv.id();

  auto sys = sv.system();

  if (sys != ConstellationEnum::GPS && sys != ConstellationEnum::GAL && sys != ConstellationEnum::QZS &&
      sys != ConstellationEnum::BDS) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "ephemeris error: invalid satellite sat=" << sv.id();

    return 0;
  }

  eph.type = NavMsgTypeMap[sv.system()];
  eph.sv = sv;
  eph.toc = toc;

  eph.f0 = data[0];
  eph.f1 = data[1];
  eph.f2 = data[2];
  eph.crs = data[4];
  eph.deln = data[5];
  eph.M0 = data[6];
  eph.cuc = data[7];
  eph.e = data[8];
  eph.cus = data[9];
  eph.sqrtA = data[10];
  eph.A = eph.sqrtA * eph.sqrtA;
  eph.toes = data[11];  // toe (s) in gps/bdt week
  eph.cic = data[12];
  eph.OMG0 = data[13];
  eph.cis = data[14];
  eph.i0 = data[15];
  eph.crc = data[16];
  eph.omg = data[17];
  eph.OMGd = data[18];
  eph.idot = data[19];
  eph.week = (i32)data[21];  // gps/bdt week
  eph.ttms = data[27];

  if (sys == ConstellationEnum::GPS || sys == ConstellationEnum::QZS) {
    eph.iode = (i32)data[3];   // IODE
    eph.iodc = (i32)data[26];  // IODC
    eph.toe = GTime(GTow(eph.toes), eph.toc);
    eph.ttm = GTime(GTow(eph.ttms), eph.toc);

    eph.code = (i32)data[20];      // GPS: codes on L2 ch
    eph.svh = (SvhEnum)data[24];   // sv health
    eph.sva = uraToSva(data[23]);  // ura (m->index)
    eph.flag = (i32)data[22];      // GPS: L2 P data flag

    eph.tgd[0] = data[25];  // TGD

    if (sys == ConstellationEnum::GPS) {
      eph.fit = data[28];
    }  // fit interval in hours for GPS
    else if (sys == ConstellationEnum::QZS) {
      eph.fitFlag = data[28];
      eph.fit = eph.fitFlag ? 0.0 : 2.0;
    }  // fit interval flag for QZS

    // if (navp::ginan::acsConfig.use_tgd_bias) decomposeTGDBias(sv, eph.tgd[0]);
  } else if (sys == ConstellationEnum::GAL) {
    // GAL ver.3
    eph.iode = (i32)data[3];  // IODnav
    eph.toe = GTime(GTow(eph.toes), eph.toc);
    eph.ttm = GTime(GTow(eph.ttms), eph.toc);

    eph.code = (i32)data[20];  // data sources
                               // bit 0 set: I/NAV E1-B
                               // bit 1 set: F/NAV E5a-I
                               // bit 2 set: I/NAV E5b-I
                               // bit 8 set: af0-af2 toc are for E5a.E1
                               // bit 9 set: af0-af2 toc are for E5b.E1
    u16 iNavMask = 0x0005;
    u16 fNavMask = 0x0002;
    if (eph.code & iNavMask)
      eph.type = NavMsgTypeEnum::INAV;
    else if (eph.code & fNavMask)
      eph.type = NavMsgTypeEnum::FNAV;

    eph.svh = (SvhEnum)data[24];  // sv health
                                  // bit     0: E1B DVS
                                  // bit   1-2: E1B HS
                                  // bit     3: E5a DVS
                                  // bit   4-5: E5a HS
                                  // bit     6: E5b DVS
                                  // bit   7-8: E5b HS
    eph.sva = sisaToSva(data[23]);

    eph.tgd[0] = data[25];  // BGD E5a/E1
    eph.tgd[1] = data[26];  // BGD E5b/E1

    // if (navp::ginan::acsConfig.use_tgd_bias) decomposeBGDBias(sv, eph.tgd[0], eph.tgd[1]);
  } else if (sys == ConstellationEnum::BDS) {
    // BeiDou v.3.02
    if (sv.prn > 5 && sv.prn < 59)
      eph.type = NavMsgTypeEnum::D1;  // MEO/IGSO
    else
      eph.type = NavMsgTypeEnum::D2;  // GEO, prn range may change in the future*/

    eph.tocs = BTow(toc);
    eph.aode = (i32)data[3];   // AODE
    eph.aodc = (i32)data[28];  // AODC
    eph.iode = i32(eph.tocs / 720) % 240;
    eph.iodc = eph.iode + 256 * i32(eph.tocs / 172800) % 4;
    eph.toe = GTime(BTow(eph.toes), eph.toc);
    eph.ttm = GTime(BTow(eph.ttms), eph.toc);

    eph.svh = (SvhEnum)data[24];   // satH1
    eph.sva = uraToSva(data[23]);  // ura (m->index)

    eph.tgd[0] = data[25];  // TGD1 B1/B3
    eph.tgd[1] = data[26];  // TGD2 B2/B3
  }

  if (eph.iode < 0 || eph.iode > 1023) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "rinex nav invalid: sat=" << sv.id() << " iode=" << eph.iode;
  }

  if (eph.iodc < 0 || eph.iodc > 1023) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "rinex nav invalid: sat=" << sv.id() << " iodc=" << eph.iodc;
  }
  return 1;
}

/** Decode glonass ephemeris
 */
i32 decodeGeph(f64 ver,                 ///< RINEX version
               Sv sv,                   ///< Satellite ID
               GTime toc,               ///< Time of clock
               std::vector<f64>& data,  ///< Data to decode
               Geph& geph)              ///< Glonass ephemeris
{
  f64 tow;

  //     BOOST_LOG_TRIVIAL(debug)
  // 	<< "decodeGeph: ver=" << ver << " sat=" << sv.id();

  if (sv.system() != ConstellationEnum::GLO) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "glonass ephemeris error: invalid satellite sat=" << sv.id();

    return 0;
  }

  geph.type = NavMsgTypeMap[sv.system()];
  geph.sv = sv;

  RTod toes = i32(RTod(toc) + 450.0) / 900 * 900.0;
  geph.toe = GTime(toes, toc);

  geph.tofs = data[2];  // UTC
  geph.tof = GTime(RTod(geph.tofs + 10800.0), toc);

  geph.iode = (i32)toes / 900;

  geph.taun = -data[0];   // -taun -> +taun
  geph.gammaN = data[1];  // +gamman

  for (i32 i = 0; i < 3; i++) {
    geph.pos[i] = data[3 + i * 4] * 1E3;
    geph.vel[i] = data[4 + i * 4] * 1E3;
    geph.acc[i] = data[5 + i * 4] * 1E3;
  }

  geph.svh = (SvhEnum)data[6];
  geph.frq = (i32)data[10];
  geph.age = (i32)data[14];

  if (ver >= 3.05) {
    // todo Eugene: additional records from version 3.05 and on
  }

  // some receiver output >128 for minus frequency number
  if (geph.frq > 128) geph.frq -= 256;

  if (geph.frq < MINFREQ_GLO || geph.frq > MAXFREQ_GLO) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "rinex gnav invalid freq: sat=" << sv << " fn=" << geph.frq;
  }
  return 1;
}

/** Decode geo ephemeris
 */
i32 decodeSeph(f64 ver, Sv sv, GTime toc, std::vector<f64>& data, Seph& seph) {
  //     BOOST_LOG_TRIVIAL(debug)
  // 	<< "decodeSeph: ver=" << ver << " sat=" << sv.id();

  if (sv.system() != ConstellationEnum::SBS) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "geo ephemeris error: invalid satellite sat=" << sv.id();

    return 0;
  }

  seph.type = NavMsgTypeMap[sv.system()];
  seph.sv = sv;
  seph.t0 = toc;

  seph.tofs = data[2];
  seph.tof = GTime(GTow(seph.tofs), seph.t0);

  seph.af0 = data[0];
  seph.af1 = data[1];

  for (i32 i = 0; i < 3; i++) {
    seph.pos[i] = data[3 + i * 4] * 1E3;
    seph.vel[i] = data[4 + i * 4] * 1E3;
    seph.acc[i] = data[5 + i * 4] * 1E3;
  }

  seph.svh = (SvhEnum)data[6];
  seph.sva = uraToSva(data[10]);

  return 1;
}

/** Decode CNVX ephemeris
 */
i32 decodeCeph(f64 ver,                 ///< RINEX version
               Sv sv,                   ///< Satellite ID
               NavMsgTypeEnum type,     ///< Navigation message type
               GTime toc,               ///< Time of clock
               std::vector<f64>& data,  ///< Data to decode
               Ceph& ceph)              ///< CNVX ephemeris
{
  //     BOOST_LOG_TRIVIAL(debug)
  // 	<< "decodeCeph: ver=" << ver << " sat=" << sv.id();

  if (ver < 4.0) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "ephemeris error: invalid RINEX version=" << ver;

    return -1;
  }

  if (type != NavMsgTypeEnum::CNAV && type != NavMsgTypeEnum::CNV1 && type != NavMsgTypeEnum::CNV2 &&
      type != NavMsgTypeEnum::CNV3) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "ephemeris error: invalid message type=" << type._to_string();

    return 0;
  }

  auto sys = sv.system();

  if (sys != ConstellationEnum::GPS && sys != ConstellationEnum::QZS && sys != ConstellationEnum::BDS) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "ephemeris error: invalid satellite sat=" << sv.id();

    return 0;
  }

  ceph.sv = sv;
  ceph.type = type;
  ceph.toc = toc;

  ceph.f0 = data[0];
  ceph.f1 = data[1];
  ceph.f2 = data[2];
  ceph.Adot = data[3];
  ceph.crs = data[4];
  ceph.deln = data[5];
  ceph.M0 = data[6];
  ceph.cuc = data[7];
  ceph.e = data[8];
  ceph.cus = data[9];
  ceph.A = data[10] * data[10];
  ceph.cic = data[12];
  ceph.OMG0 = data[13];
  ceph.cis = data[14];
  ceph.i0 = data[15];
  ceph.crc = data[16];
  ceph.omg = data[17];
  ceph.OMGd = data[18];
  ceph.idot = data[19];
  ceph.dn0d = data[20];

  if (sys == ConstellationEnum::GPS || sys == ConstellationEnum::QZS) {
    ceph.toe = ceph.toc;
    ceph.toes = GTow(ceph.toe);

    ceph.ura[0] = data[21];
    ceph.ura[1] = data[22];
    ceph.ura[2] = data[26];
    ceph.ura[3] = data[23];

    ceph.svh = (SvhEnum)data[24];  // sv health

    ceph.tgd[0] = data[25];  // TGD

    ceph.isc[0] = data[27];
    ceph.isc[1] = data[28];
    ceph.isc[2] = data[29];
    ceph.isc[3] = data[30];

    if (type == NavMsgTypeEnum::CNAV) {
      ceph.ttms = data[31];
      ceph.ttm = GTime(GTow(ceph.ttms), ceph.toc);
      ceph.wnop = (i32)data[32];
    } else if (type == NavMsgTypeEnum::CNV2) {
      ceph.isc[4] = data[31];
      ceph.isc[5] = data[32];

      ceph.ttms = data[35];
      ceph.ttm = GTime(GTow(ceph.ttms), ceph.toc);
      ceph.wnop = (i32)data[36];
    }

    ceph.tops = data[11];  // top (s) in seconds
    ceph.top = GTime(GTow(ceph.tops), ceph.toc);
  } else if (sys == ConstellationEnum::BDS) {
    // BeiDou v.4.00

    ceph.orb = magic_enum::enum_cast<SatTypeEnum>(data[21]).value();

    ceph.sis[0] = data[23];
    ceph.sis[1] = data[24];
    ceph.sis[2] = data[25];
    ceph.sis[3] = data[26];

    if (type == NavMsgTypeEnum::CNV1 || type == NavMsgTypeEnum::CNV2) {
      ceph.isc[0] = data[27];
      ceph.isc[1] = data[28];

      ceph.tgd[0] = data[29];  // TGD_B1Cp
      ceph.tgd[1] = data[30];  // TGD_B2ap

      ceph.sis[4] = data[31];

      ceph.svh = (SvhEnum)data[32];  // sv health
      ceph.flag = (i32)data[33];     // integrity flag
      ceph.iodc = (i32)data[34];     // IODC
      ceph.iode = (i32)data[38];     // IODE

      ceph.ttms = data[35];
      ceph.ttm = GTime(BTow(ceph.ttms), ceph.toc);
    } else if (type == NavMsgTypeEnum::CNV3) {
      ceph.sis[4] = data[27];
      ceph.svh = (SvhEnum)data[28];  // sv health
      ceph.flag = (i32)data[29];     // integrity flag
      ceph.tgd[2] = data[30];        // TGD_B2ap

      ceph.ttms = data[31];
      ceph.ttm = GTime(BTow(ceph.ttms), ceph.toc);
    }

    ceph.toes = data[11];  // top (s) in seconds
    ceph.tops = data[22];  // top (s) in seconds
    ceph.toe = GTime(BTow(ceph.toes), ceph.toc);
    ceph.top = GTime(BTow(ceph.tops), ceph.toc);
  }

  if (ceph.iode < 0 || ceph.iode > 1023) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "rinex nav invalid: sat=" << sv.id() << " iode=" << ceph.iode;
  }

  if (ceph.iodc < 0 || ceph.iodc > 1023) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "rinex nav invalid: sat=" << sv.id() << " iodc=" << ceph.iodc;
  }

  return 1;
}

/** Decode STO message
 */
i32 decodeSto(f64 ver, Sv sv, NavMsgTypeEnum type, GTime toc, std::vector<f64>& data, STO& sto) {
  if (ver < 4.0) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "ephemeris error: invalid RINEX version=" << ver;

    return -1;
  }

  auto sys = sv.system();

  sto.sv = sv;
  sto.type = type;
  sto.tot = toc;

  sto.code = magic_enum::enum_cast<StoCodeEnum>(data[0]).value();
  sto.sid = magic_enum::enum_cast<SbasIdEnum>(data[1]).value();
  sto.uid = magic_enum::enum_cast<UtcIdEnum>(data[2]).value();

  sto.ttms = data[3];

  sto.A0 = data[4];
  sto.A1 = data[5];
  sto.A2 = data[6];

  if (sys != ConstellationEnum::BDS) {
    sto.ttm = GTime(GWeek(sto.tot), GTow(sto.ttms));
  } else {
    sto.ttm = GTime(BWeek(sto.tot), BTow(sto.ttms));
  }

  return 1;
}

/** Decode EOP message
 */
i32 decodeEop(f64 ver, Sv sv, NavMsgTypeEnum type, GTime toc, std::vector<f64>& data, EOP& eop) {
  if (ver < 4.0) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "ephemeris error: invalid RINEX version=" << ver;

    return -1;
  }

  auto sys = sv.system();

  eop.sv = sv;
  eop.type = type;
  eop.teop = toc;

  eop.xp = data[0] * AS2R;
  eop.xpr = data[1] * AS2R;
  eop.xprr = data[2] * AS2R;
  eop.yp = data[4] * AS2R;
  eop.ypr = data[5] * AS2R;
  eop.yprr = data[6] * AS2R;
  eop.ttms = data[7];
  eop.dut1 = data[8];
  eop.dur = data[9];
  eop.durr = data[10];

  if (sys != ConstellationEnum::BDS) {
    eop.ttm = GTime(GWeek(eop.teop), GTow(eop.ttms));
  } else {
    eop.ttm = GTime(BWeek(eop.teop), BTow(eop.ttms));
  }

  return 1;
}

/** Decode ION message
 */
i32 decodeIon(f64 ver, Sv sv, NavMsgTypeEnum type, GTime toc, std::vector<f64>& data, ION& ion) {
  if (ver < 4.0) {
    // BOOST_LOG_TRIVIAL(debug)
    // << "ephemeris error: invalid RINEX version=" << ver;

    return -1;
  }

  auto sys = sv.system();

  ion.sv = sv;
  ion.type = type;
  ion.ttm = toc;

  if (sys == ConstellationEnum::GAL && type == NavMsgTypeEnum::IFNV) {
    ion.ai0 = data[0];
    ion.ai1 = data[1];
    ion.ai2 = data[2];

    ion.flag = (i32)data[3];
  } else if (sys == ConstellationEnum::BDS && type == NavMsgTypeEnum::CNVX) {
    ion.alpha1 = data[0];
    ion.alpha2 = data[1];
    ion.alpha3 = data[2];
    ion.alpha4 = data[3];
    ion.alpha5 = data[4];
    ion.alpha6 = data[5];
    ion.alpha7 = data[6];
    ion.alpha8 = data[7];
    ion.alpha9 = data[8];
  } else if (type == NavMsgTypeEnum::LNAV || type == NavMsgTypeEnum::D1D2 || type == NavMsgTypeEnum::CNVX) {
    ion.a0 = data[0];
    ion.a1 = data[1];
    ion.a2 = data[2];
    ion.a3 = data[3];
    ion.b0 = data[4];
    ion.b1 = data[5];
    ion.b2 = data[6];
    ion.b3 = data[7];

    ion.code = (i32)data[8];

    if (ion.code == 1)  // QZS Japan area coefficients are currently skipped
      return 0;
  }

  return 1;
}

/** Read rinex navigation data body
 */
i32 readRnxNavB(std::istream& inputStream,  ///< Input stream to read
                f64 ver,                    ///< RINEX version
                ConstellationEnum sys,      ///< Satellite system
                EphemerisType& type,        ///< Ephemeris type (output)
                Eph& eph,                   ///< GPS Ephemeris
                Geph& geph,                 ///< Glonass ephemeris
                Seph& seph,                 ///< Geo ephemeris
                Ceph& ceph,                 ///< CNVX ephemeris
                STO& sto,                   ///< System time offset data
                EOP& eop,                   ///< EOP data
                ION& ion)                   ///< Ionosphere data
{
  GTime toc;
  std::vector<f64> data;
  i32 sp = 3;
  string line;
  char id[8] = "";
  char* p;

  // 	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << ": ver=" << ver << " sys=" << sys;

  Sv sv = {};
  NavRecTypeEnum recType = NavRecTypeEnum::NONE;
  NavMsgTypeEnum msgType = NavMsgTypeEnum::NONE;

  while (std::getline(inputStream, line)) {
    char* buff = &line[0];

    if (data.empty()) {
      // decode message type field
      if (ver >= 4.0 && buff[0] == '>') {
        // ver.4
        char typeStr[5] = "";
        strncpy(typeStr, buff + 2, 3);
        recType = magic_enum::enum_cast<NavRecTypeEnum>(typeStr).value();

        strncpy(id, buff + 6, 3);
        sv = Sv::from_str(id).unwrap();
        sys = sv.system();

        strncpy(typeStr, buff + 10, 4);
        std::replace(typeStr, typeStr + 4, ' ', '\0');
        msgType = magic_enum::enum_cast<NavMsgTypeEnum>(typeStr).value();

        continue;
      }

      // decode satellite field
      if (ver >= 3.0 || sys == ConstellationEnum::GAL || sys == ConstellationEnum::QZS) {
        // ver.3 or GAL/QZS
        strncpy(id, buff, 3);
        sp = 4;
        if (ver < 4.0)  // satellite id included in message type field in ver.4
        {
          sv = Sv::from_str(id).unwrap();
          if (ver >= 3.0) sys = sv.system();
        }
      } else {
        sv.system() = sys;
        sv.prn = str2num(buff, 0, 2);
      }

      TimeSystemEnum tsys = TimeSystemEnum::GPST;
      switch (sys) {
        case ConstellationEnum::GPS:
          tsys = TimeSystemEnum::GPST;
          break;
        case ConstellationEnum::GLO:
          tsys = TimeSystemEnum::UTC;
          break;
        case ConstellationEnum::GAL:
          tsys = TimeSystemEnum::GST;
          break;
        case ConstellationEnum::BDS:
          tsys = TimeSystemEnum::BDT;
          break;
        case ConstellationEnum::QZS:
          tsys = TimeSystemEnum::QZSST;
          break;
        case ConstellationEnum::SBS:
          tsys = TimeSystemEnum::GPST;
          break;
        default:
          tsys = TimeSystemEnum::GPST;
          break;
      }

      // decode toc field
      bool error = str2time(buff + sp, 0, 19, toc, tsys);
      if (error == true) {
        //                 BOOST_LOG_TRIVIAL(debug)
        // 				<< "rinex nav toc error: " << buff;

        return 0;
      }

      if (recType == NavRecTypeEnum::STO) {
        // decode STO code, SBAS ID & UTC ID for STO message
        char code[19] = "";
        strncpy(code, buff + 24, 18);
        std::replace(code, code + 18, ' ', '\0');
        data.emplace_back(static_cast<f64>(magic_enum::enum_cast<StoCodeEnum>(code).value()));

        strncpy(code, buff + 43, 18);
        std::replace(code, code + 18, '-', '_');
        std::replace(code, code + 18, ' ', '\0');
        data.emplace_back(
            static_cast<f64>(magic_enum::enum_cast<SbasIdEnum>(code).value_or(SbasIdEnum::NONE)));  // code may be empty

        strncpy(code, buff + 62, 18);
        std::replace(code, code + 18, '(', '_');
        std::replace(code, code + 18, ')', '\0');
        std::replace(code, code + 18, ' ', '\0');
        data.emplace_back(
            static_cast<f64>(magic_enum::enum_cast<UtcIdEnum>(code).value_or(UtcIdEnum::NONE)));  // code may be empty
      } else {
        // decode data fields
        p = buff + sp + 19;
        for (i32 j = 0; j < 3; j++, p += 19) {
          data.emplace_back(str2num(p, 0, 19));
        }
      }

      if (recType == NavRecTypeEnum::NONE) recType = NavRecTypeEnum::EPH;
      if (msgType == NavMsgTypeEnum::NONE) msgType = NavMsgTypeMap[sys];
    } else {
      // decode data fields
      p = buff + sp;
      for (i32 j = 0; j < 4; j++, p += 19) {
        data.emplace_back(str2num(p, 0, 19));
      }
      // decode ephemeris
      if (recType == NavRecTypeEnum::EPH) {
        switch (msgType) {
          case NavMsgTypeEnum::CNAV:  // fallthrough
          case NavMsgTypeEnum::CNV3: {
            if (data.size() >= 35) {
              type = EphemerisType::CEPH;
              return decodeCeph(ver, sv, msgType, toc, data, ceph);
            }
            break;
          }
          case NavMsgTypeEnum::CNV1:  // fallthrough
          case NavMsgTypeEnum::CNV2: {
            if (data.size() >= 39) {
              type = EphemerisType::CEPH;
              return decodeCeph(ver, sv, msgType, toc, data, ceph);
            }
            break;
          }
          case NavMsgTypeEnum::FDMA: {
            if (data.size() >= 15) {
              type = EphemerisType::GEPH;
              return decodeGeph(ver, sv, toc, data, geph);
            }
            break;
          }  // todo Eugene: additional records from version 3.05 and on
          case NavMsgTypeEnum::SBAS: {
            if (data.size() >= 15) {
              type = EphemerisType::SEPH;
              return decodeSeph(ver, sv, toc, data, seph);
            }
            break;
          }
          default: {
            if (data.size() >= 31) {
              type = EphemerisType::EPH;
              return decodeEph(ver, sv, toc, data, eph);
            }
            break;
          }
        }
      } else if (recType == NavRecTypeEnum::STO) {
        if (data.size() >= 7) {
          type = EphemerisType::STO;
          return decodeSto(ver, sv, msgType, toc, data, sto);
        }
      } else if (recType == NavRecTypeEnum::EOP) {
        if (data.size() >= 11) {
          type = EphemerisType::EOP;
          return decodeEop(ver, sv, msgType, toc, data, eop);
        }
      } else if (recType == NavRecTypeEnum::ION) {
        switch (sys) {
          case ConstellationEnum::GAL: {
            if (data.size() >= 7) {
              type = EphemerisType::ION;
              return decodeIon(ver, sv, msgType, toc, data, ion);
            }
            break;
          }
          default: {
            if (data.size() >= 11) {
              type = EphemerisType::ION;
              return decodeIon(ver, sv, msgType, toc, data, ion);
            }
            break;
          }
        }
      } else
        return -1;
    }
  }
  return -1;
}

/** Read rinex nav/gnav/geo nav
 */
i32 readRnxNav(std::istream& inputStream,  ///< Input stream to read
               f64 ver,                    ///< RINEX version
               ConstellationEnum sys,      ///< Satellite system
               Navigation& nav)            ///< Navigation object
{
  // 	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << ": ver=" << ver << " sys=" << sys;

  // read rinex navigation data body
  while (1) {
    // initialise each time to avoid incomplete overwriting
    Eph eph = {};
    Geph geph = {};
    Seph seph = {};
    Ceph ceph = {};
    STO sto = {};
    EOP eop = {};
    ION ion = {};

    EphemerisType type;

    i32 stat = readRnxNavB(inputStream, ver, sys, type, eph, geph, seph, ceph, sto, eop, ion);

    if (stat < 0) {
      break;
    } else if (stat > 0) {
      // add ephemeris to navigation data
      switch (type) {
        case EphemerisType::EPH:
          nav.ephMap[eph.sv][eph.type][eph.toe] = eph;
          break;
        case EphemerisType::GEPH:
          nav.gephMap[geph.sv][geph.type][geph.toe] = geph;
          break;
        case EphemerisType::SEPH:
          nav.sephMap[seph.sv][seph.type][seph.t0] = seph;
          break;
        case EphemerisType::CEPH:
          nav.cephMap[ceph.sv][ceph.type][ceph.toe] = ceph;
          break;
        case EphemerisType::STO:
          nav.stoMap[sto.code][sto.type][sto.tot] = sto;
          break;
        case EphemerisType::EOP:
          nav.eopMap[eop.sv.system()][eop.type][eop.teop] = eop;
          break;
        case EphemerisType::ION:
          nav.ionMap[ion.sv.system()][ion.type][ion.ttm] = ion;
          break;
        default:
          continue;
      }
    }
  }

  for (auto& [sys, eopSysMap] : nav.eopMap)
    for (auto& [type, eopList] : eopSysMap) {
      std::map<GTime, ERPValues> erpMap;

      for (auto& [time, eop] : eopList) {
        ERPValues erpv;
        erpv.time = eop.teop;
        erpv.xp = eop.xp;
        erpv.yp = eop.yp;
        erpv.ut1Utc = eop.dut1;
        erpv.xpr = eop.xpr;
        erpv.ypr = eop.ypr;
        erpMap[erpv.time] = erpv;
      }

      if (!erpMap.empty()) nav.erp.erpMaps.emplace_back(std::move(erpMap));
    }

  return (nav.ephMap.empty() == false || nav.gephMap.empty() == false || nav.sephMap.empty() == false ||
          nav.cephMap.empty() == false || nav.stoMap.empty() == false || nav.eopMap.empty() == false ||
          nav.ionMap.empty() == false);
}

/** Read rinex clock
 */
i32 readRnxClk(std::istream& inputStream, f64 ver, Navigation& nav) {
  //     trace(3,"readrnxclk: index=%d\n", index);

  static i32 index = 0;
  index++;
  string line;

  typedef struct {
    i16 offset;
    i16 length;
  } ClkStruct;

  ClkStruct typ = {0, 2};
  ClkStruct as = {3, 3};
  ClkStruct ar = {3, 4};
  ClkStruct tim = {8, 26};
  ClkStruct clk = {40, 19};
  ClkStruct std = {60, 19};

  // special case for 3.04 rnx with 9 char AR names
  if (ver == 3.04) {
    ar.length += 5;
    tim.offset += 5;
    clk.offset += 5;
    std.offset += 5;
  }

  while (std::getline(inputStream, line)) {
    char* buff = &line[0];

    GTime time;
    if (str2time(buff, tim.offset, tim.length, time)) {
      //             trace(2,"rinex clk invalid epoch: %34.34s\n", buff);
      continue;
    }

    string type(buff + typ.offset, typ.length);

    string idString;
    if (type == "AS") {
      idString.assign(buff + as.offset, as.length);
    } else if (type == "AR") {
      idString.assign(buff + ar.offset, ar.length);
    } else
      continue;

    Pclk preciseClock = {};

    preciseClock.clk = str2num(buff, clk.offset, clk.length);
    preciseClock.clkStd = str2num(buff, std.offset, std.length);
    preciseClock.clkIndex = index;

    nav.pclkMap[idString][time] = preciseClock;
  }

  return nav.pclkMap.size() > 0;
}

/** Read rinex file
 */
i32 readRnx(std::istream& inputStream, char& type, ObsList& obsList, Navigation& nav, RinexStation& rnxRec, f64& ver,
            ConstellationEnum& sys, TimeSystemEnum& tsys,
            std::map<ConstellationEnum, std::map<i32, CodeType>>& sysCodeTypes, char* glo_fcn, f64* glo_cpbias) {
  if (inputStream.tellg() == 0) {
    // read rinex header if at beginning of file
    readRnxH(inputStream, ver, type, sys, tsys, sysCodeTypes, nav, rnxRec, glo_fcn, glo_cpbias);
  }

  // read rinex body
  switch (type) {
    case 'O':
      return readRnxObs(inputStream, ver, tsys, sysCodeTypes, obsList, rnxRec);
    case 'N':
      return readRnxNav(inputStream, ver, sys, nav);
    case 'G':
      return readRnxNav(inputStream, ver, ConstellationEnum::GLO, nav);
    case 'H':
      return readRnxNav(inputStream, ver, ConstellationEnum::SBS, nav);
    case 'J':
      return readRnxNav(inputStream, ver, ConstellationEnum::QZS, nav);  // extension
    case 'L':
      return readRnxNav(inputStream, ver, ConstellationEnum::GAL, nav);  // extension
    case 'C':
      return readRnxClk(inputStream, ver, nav);
    default: {
      nav_error("Not implmentted");
    }
  }

  // BOOST_LOG_TRIVIAL(debug)
  // << "unsupported rinex type ver=" << ver << " type=" << type;

  return 0;
}
}  // namespace navp::io::rinex
