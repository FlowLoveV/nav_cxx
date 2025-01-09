#pragma once

#include <Eigen/Eigen>

#include "sensors/gnss/enums.hpp"
#include "sensors/gnss/sv.hpp"
#include "utils/eigen.hpp"
#include "utils/gTime.hpp"

namespace navp::ginan {

using sensors::gnss::ObsCodeEnum;
using sensors::gnss::Sv;
using sensors::gnss::TrigTypeEnum;

struct SatPos;

const f64 uraSsr[] = {0,       0.25,   0.5,     0.75,   1,       1.25,   1.5,     1.75,    2,      2.75,    3.5,
                      4.25,    5,      5.75,    6.5,    7.25,    8,      10.25,   12.5,    14.75,  17,      19.25,
                      21.5,    23.75,  26,      32.75,  39.5,    46.25,  53,      59.75,   66.5,   73.25,   80,
                      100.25,  120.5,  140.75,  161,    181.25,  201.5,  221.75,  242,     302.75, 363.5,   424.25,
                      485,     545.75, 606.5,   667.25, 728,     910.25, 1092.5,  1274.75, 1457,   1639.25, 1821.5,
                      2003.75, 2186,   2732.75, 3279.5, 3826.25, 4373,   4919.75, 5466.5,  6013.25};

// SSR message metadata
struct SSRMeta {
  i32 epochTime1s = 0;
  utils::GTime receivedTime = {};
  i32 updateIntIndex = -1;
  i32 multipleMessage = 0;
  u32 referenceDatum = 0;
  u32 provider = 0;
  u32 solution = 0;
  u32 numSats = 0;
};

struct SSREph {
  SSRMeta ssrMeta = {};
  utils::GTime t0 = {};
  f64 udi = 0;  ///< update interval
  i32 iod = -1;
  i32 iode = -1;  ///< issue of data
  i32 iodcrc = -1;
  utils::NavVector3f64 deph = utils::NavVector3f64::Zero();   ///<     delta orbit {radial,along,cross} (m)
  utils::NavVector3f64 ddeph = utils::NavVector3f64::Zero();  ///< dot delta orbit {radial,along,cross} (m/s)
};

struct SSRClk {
  SSRMeta ssrMeta = {};
  utils::GTime t0 = {};
  f64 udi = 0;  ///< update interval
  i32 iod = -1;
  f64 dclk[3] = {};  ///< delta clock {c0,c1,c2} (m,m/s,m/s^2)
};

struct SSRUra {
  SSRMeta ssrMeta = {};
  utils::GTime t0 = {};
  f64 udi = 0;  ///< update interval
  i32 iod = -1;
  i32 ura = 0;  ///< URA indicator
};

struct SSRHRClk {
  SSRMeta ssrMeta = {};
  utils::GTime t0 = {};
  f64 udi = 0;  ///< update interval
  i32 iod = -1;
  f64 hrclk = 0;  ///< high-rate clock corection (m)
};

struct BiasVar {
  f64 bias = 0;  ///< biases (m)
  f64 var = 0;   ///< biases variance (m^2)
};

struct SSRBias {
  SSRMeta ssrMeta = {};
  utils::GTime t0 = {};
  f64 udi = 0;  ///< update interval
  i32 iod = -1;
  u32 nbias = 0;
  std::map<ObsCodeEnum, BiasVar> obsCodeBiasMap;
  std::map<i32, f64> ionDCBOffset;
};

struct SSRCodeBias : SSRBias {};

struct SSRPhase {
  i32 dispBiasConistInd = -1;
  i32 MWConistInd = -1;
  f64 yawAngle = 0;
  f64 yawRate = 0;
};

struct SSRPhaseCh {
  u32 signalIntInd = -1;
  u32 signalWLIntInd = -1;
  u32 signalDisconCnt = -1;
};

struct SSRPhasBias : SSRBias {
  SSRPhase ssrPhase;                              ///< Additional data for SSR phase messages
  std::map<ObsCodeEnum, SSRPhaseCh> ssrPhaseChs;  ///< Additional data for SSR phase messages, for each channel
};

struct SphComp {
  i32 layer;
  i32 order;
  i32 degree;
  TrigTypeEnum trigType;
  f64 value;
  f64 variance;
};

struct SSRVTEClayer {
  f64 height = 0;
  i32 maxOrder = 0;
  i32 maxDegree = 0;
  std::map<i32, SphComp> sphHarmonic;
};

struct SSRAtmGlobal {
  utils::GTime time;
  i32 numberLayers;
  std::map<i32, SSRVTEClayer> layers;
  f64 vtecQuality;
  i32 iod = -1;
};

struct SSRSTECData {
  i32 iod = -1;
  f64 sigma = 0.1;         /* STEC std::maps accuracy in TECu */
  std::map<i32, f64> poly; /* STEC polynomials in TECu (deg) */
  std::map<i32, f64> grid; /* STEC gridstd::maps in TECu */
};

struct SSRTropData {
  f64 sigma = 0;
  std::map<i32, f64> polyDry; /* ZHD in meters (deg) */
  std::map<i32, f64> gridDry; /* ZHD in meters */
  std::map<i32, f64> gridWet; /* ZWD in meters */
};

struct SSRAtmRegion {
  i32 regionDefIOD = -1;
  std::map<i32, f64> gridLatDeg;
  std::map<i32, f64> gridLonDeg;

  f64 minLatDeg = 0;
  f64 maxLatDeg = 0;
  f64 intLatDeg = 0;

  f64 minLonDeg = 0;
  f64 maxLonDeg = 0;
  f64 intLonDeg = 0;

  i32 gridType = -1;
  i32 tropPolySize = -1;
  i32 ionoPolySize = -1;
  bool ionoGrid = false;
  bool tropGrid = false;

  std::map<utils::GTime, SSRTropData, std::greater<utils::GTime>> tropData;
  std::map<Sv, std::map<utils::GTime, SSRSTECData, std::greater<utils::GTime>>> stecData;
  utils::GTime stecUpdateTime;
};

struct SSRAtm {
  SSRMeta ssrMeta;
  std::map<utils::GTime, SSRAtmGlobal, std::greater<utils::GTime>> atmosGlobalMap;
  std::map<i32, SSRAtmRegion> atmosRegionsMap;
};

struct EphValues {
  utils::GTime time;
  u32 iode = -1;
  utils::NavVector3f64 brdcPos = utils::NavVector3f64::Zero();
  utils::NavVector3f64 brdcVel = utils::NavVector3f64::Zero();
  utils::NavVector3f64 precPos = utils::NavVector3f64::Zero();
  utils::NavVector3f64 precVel = utils::NavVector3f64::Zero();

  f64 ephVar = 0;
};

struct ClkValues {
  utils::GTime time;
  u32 iode = -1;
  f64 brdcClk = 0;
  f64 precClk = 0;
};

struct SSREphInput {
  bool valid = false;
  EphValues vals[2];
};

struct SSRClkInput {
  bool valid = false;
  ClkValues vals[2];
};

/* SSR correction type */
struct SSRMaps {
  std::map<utils::GTime, SSRCodeBias, std::greater<utils::GTime>> ssrCodeBias_map;
  std::map<utils::GTime, SSRPhasBias, std::greater<utils::GTime>> ssrPhasBias_map;
  std::map<utils::GTime, SSRClk, std::greater<utils::GTime>> ssrClk_map;
  std::map<utils::GTime, SSREph, std::greater<utils::GTime>> ssrEph_map;
  std::map<utils::GTime, SSRHRClk, std::greater<utils::GTime>> ssrHRClk_map;
  std::map<utils::GTime, SSRUra, std::greater<utils::GTime>> ssrUra_map;

  i32 refd_;   ///< sat ref datum (0:ITRF,1:regional)
  u8 update_;  ///< update flag (0:no update,1:update)
};

struct SSROut {
  utils::GTime epochTime;

  SSRPhasBias ssrPhasBias;
  SSRCodeBias ssrCodeBias;

  SSRClkInput clkInput;
  SSREphInput ephInput;

  SSRClk ssrClk;
  SSREph ssrEph;

  SSRHRClk ssrHRClk;
  SSRUra ssrUra;

  bool ephUpdated = false;
  bool clkUpdated = false;
  bool hrclkUpdated = false;
  bool phaseUpdated = false;
  bool codeUpdated = false;
  bool uraUpdated = false;
};
};  // namespace navp::ginan