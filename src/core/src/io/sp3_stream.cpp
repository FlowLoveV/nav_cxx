#include "io/sp3/sp3_stream.hpp"

#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/ephemeris.hpp"
#include "sensors/gnss/navigation.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/gTime.hpp"

using navp::sensors::gnss::ConstellationEnum;
using navp::sensors::gnss::GnssNavRecord;
using navp::sensors::gnss::Peph;
using navp::sensors::gnss::Sv;
using navp::sensors::gnss::TimeSystemEnum;
using navp::utils::GTime;

using navp::utils::str2num;
/** satellite code to satellite system
 */
ConstellationEnum code2sys(char code) {
  if (code == 'G' || code == ' ') return ConstellationEnum::GPS;
  if (code == 'R') return ConstellationEnum::GLO;
  if (code == 'E') return ConstellationEnum::GAL; /* extension to sp3-c */
  if (code == 'J') return ConstellationEnum::QZS; /* extension to sp3-c */
  if (code == 'C') return ConstellationEnum::BDS; /* extension to sp3-c */
  if (code == 'L') return ConstellationEnum::LEO; /* extension to sp3-c */
  return ConstellationEnum::NONE;
}

namespace navp::io::sp3 {

// read function
bool readsp3(std::istream& fileStream,     ///< stream to read content from
             std::vector<Peph>& pephList,  ///< vector of precise ephemerides for one epoch
             i32 opt,                      ///< options options (1: only observed + 2: only predicted + 4: not combined)
             TimeSystemEnum& tsys,         ///< time system
             f64* bfact)                   ///< bfact values from header
{
  GTime time = {};

  // keep track of file number
  static i32 index = 0;
  index++;

  i32 hashCount = 0;
  i32 cCount = 0;
  i32 fCount = 0;

  bool epochFound = false;
  std::string line;
  while (fileStream) {
    // return early when an epoch is complete
    i32 peek = fileStream.peek();
    if (peek == '*' && epochFound) {
      return true;
    }

    getline(fileStream, line);

    char* buff = &line[0];

    if (buff[0] == '*') {
      // epoch line
      epochFound = true;

      bool error = str2time(buff, 3, 28, time, tsys);
      if (error) {
        printf("\nInvalid epoch line in sp3 file %s\n", line.c_str());
        return false;
      }

      continue;
    }

    if (buff[0] == 'P') {
      // position line
      bool pred_p = false;
      bool pred_c = false;

      ConstellationEnum sys = code2sys(buff[1]);
      i32 prn = (i32)str2num(buff, 2, 2);

      Sv sv{.prn = (navp::u8)prn, .constellation = {.id = sys}};
      if (!sv) continue;

      Peph peph = {};
      peph.time = time;
      peph.index = index;
      peph.sv = sv;
      bool valid = true;

      if (buff[0] == 'P') {
        pred_c = strlen(buff) >= 76 && buff[75] == 'P';
        pred_p = strlen(buff) >= 80 && buff[79] == 'P';
      }

      // positions/rates
      for (i32 j = 0; j < 3; j++) {
        /* read option for predicted value */
        if (j < 3 && (opt & 1) && pred_p) continue;
        if (j < 3 && (opt & 2) && !pred_p) continue;

        f64 val = str2num(buff, 4 + j * 14, 14);
        f64 std = str2num(buff, 61 + j * 3, 2);

        if (buff[0] == 'P') {
          /* position */
          if (val != 0) {
            peph.pos[j] = val * 1000;
          } else {
            valid = false;
          }

          f64 base = bfact[0];
          if (base > 0 && std > 0) {
            peph.posStd[j] = pow(base, std) * 1E-3;
          }
        } else if (valid) {
          // 					/* velocity */
          // 					if	( val !=0)
          // 					{
          // 						peph.vel[j] = val * 0.1;
          // 					}
          //
          // 					f64 base = bfact[j < 3 ? 0 : 1];
          // 					if	(  base	> 0
          // 						&& std	> 0)
          // 					{
          // 						peph.velStd[j] = pow(base, std) * 1E-7;
          // 					}
        }
      }

      // clocks / rates
      for (i32 j = 3; j < 4; j++) {
        /* read option for predicted value */
        if (j == 3 && (opt & 1) && pred_c) continue;
        if (j == 3 && (opt & 2) && !pred_c) continue;

        std::string checkValue;
        checkValue.assign(buff + 4 + j * 14, 7);
        f64 val = str2num(buff, 4 + j * 14, 14);
        f64 std = str2num(buff, 61 + j * 3, 3);

        if (buff[0] == 'P') {
          /* clock */
          if (val != 0 && checkValue != " 999999") {
            peph.clk = val * 1E-6;
          } else {
            peph.clk = INVALID_CLOCK_VALUE;
            // 						valid = false;	//allow clocks to be invalid
          }

          f64 base = bfact[1];
          if (base > 0 && std > 0) {
            peph.clkStd = pow(base, std) * 1E-12;
          }
        } else if (valid) {
          // 					/* clock rate */
          // 					if	( val !=0
          // 						&&fabs(val) < NO_SP3_CLK)
          // 					{
          // 						peph.dCk = val * 1E-10;
          // 					}
          //
          // 					f64 base = bfact[j < 3 ? 0 : 1];
          // 					if	(  base	> 0
          // 						&& std	> 0)
          // 					{
          // 						peph.dCkStd = pow(base, std) * 1E-16;
          // 					}
        }
      }

      if (valid) {
        pephList.push_back(peph);
      }

      continue;
    }
    /*
Quick and dirty read of the velocities
@todo change later.
    */
    if (buff[0] == 'V') {
      for (i32 i = 0; i < 3; ++i) {
        f64 val = str2num(buff, 4 + i * 14, 14);
        pephList.back().vel[i] = val * 0.1;
      }
    }
    if (buff[0] == '#') {
      hashCount++;

      if (hashCount == 1) {
        // first line is time and type
        // 				type = buff[2];
        i32 error = str2time(buff, 3, 28, time);  // time system unknown at beginning but does not matter
        if (error) return false;

        continue;
      }
    }

    std::string twoChars = line.substr(0, 2);

    // 		if (twoChars == "+ ")
    // 		{
    //			plusCount++;
    // 			//number and list of satellites included in the file - information only, sat ids are included in
    // epoch lines.. 			if (lineNum == 2)
    // 			{
    // 				ns = (int)str2num(buff,4,2);
    // 			}
    // 			for (int j = 0; j < 17 && k < ns; j++)
    // 			{
    // 				E_Sys sys=code2sys(buff[9+3*j]);
    //
    // 				int prn = (int)str2num(buff,10+3*j,2);
    //
    // 				if (k < MAXSAT)
    // 					sats[k++] = SatSys(sys, prn);
    // 			}
    //			continue;
    // 		}

    // 		if (twoChars == "++")
    // 		{
    // 			pplusCount++;
    // 			continue;
    // 		}

    if (twoChars == "%c") {
      cCount++;

      if (cCount == 1) {
        std::string timeSysStr = line.substr(9, 3);

        if (timeSysStr == "GPS")
          tsys = TimeSystemEnum::GPST;
        else if (timeSysStr == "GLO")
          tsys = TimeSystemEnum::GLONASST;
        else if (timeSysStr == "GAL")
          tsys = TimeSystemEnum::GST;
        else if (timeSysStr == "QZS")
          tsys = TimeSystemEnum::QZSST;
        else if (timeSysStr == "TAI")
          tsys = TimeSystemEnum::TAI;
        else if (timeSysStr == "UTC")
          tsys = TimeSystemEnum::UTC;
        else {
          return false;
        }
      }
      continue;
    }

    if (twoChars == "%f") {
      fCount++;

      if (fCount == 1) {
        bfact[0] = str2num(buff, 3, 10);
        bfact[1] = str2num(buff, 14, 12);
      }
      continue;
    }

    if (line.substr(0, 3) == "EOF") {
      return true;
    }
  }

  return false;
}

void Sp3Stream::decode_record(Record& record) {
  if (auto nav_record = dynamic_cast<GnssNavRecord*>(&record); nav_record) {
    std::vector<Peph> eph_vec;
    TimeSystemEnum tsys;
    f64 bfact[2];
    readsp3(*this, eph_vec, 0, tsys, bfact);
    std::ranges::for_each(eph_vec, [&](const Peph& peh) { nav_record->nav->pephMap[peh.sv][peh.time] = peh; });
  } else {
    nav_warn("Sp3Stream decode unmatched record");
  }
};

void Sp3Stream::encode_record(const Record& record) {
  nav_error("not implmentted");
  exit(-1);
}

}  // namespace navp::io::sp3
