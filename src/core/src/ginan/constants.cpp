#include "ginan/constants.hpp"

#define CLIGHT 299792458.0   /* speed of light (m/s) */
#define FREQ1 1.57542E9      /* L1/E1  frequency (Hz) */
#define FREQ2 1.22760E9      /* L2     frequency (Hz) */
#define FREQ5 1.17645E9      /* L5/E5a frequency (Hz) */
#define FREQ6 1.27875E9      /* E6/LEX frequency (Hz) */
#define FREQ7 1.20714E9      /* E5b    frequency (Hz) */
#define FREQ8 1.191795E9     /* E5a+b  frequency (Hz) */
#define FREQ1_GLO 1.60200E9  /* GLONASS G1 base frequency (Hz) */
#define DFRQ1_GLO 0.56250E6  /* GLONASS G1 bias frequency (Hz/n) */
#define FREQ2_GLO 1.24600E9  /* GLONASS G2 base frequency (Hz) */
#define DFRQ2_GLO 0.43750E6  /* GLONASS G2 bias frequency (Hz/n) */
#define FREQ3_GLO 1.202025E9 /* GLONASS G3 frequency (Hz) */
#define FREQ4_GLO 1.600995E9 /* GLONASS G4 (CDMA G1) frequency (Hz) */
#define FREQ6_GLO 1.248060E9 /* GLONASS G6 (CDMA G2) frequency (Hz) */
#define FREQ1_CMP 1.561098E9 /* BeiDou B1 frequency (Hz) */
#define FREQ2_CMP 1.207140E9 /* BeiDou B2 frequency (Hz) */
#define FREQ3_CMP 1.268520E9 /* BeiDou B3 frequency (Hz) */
#define FREQ9_IRN 2.492028E9 /* NavIC / IRNSS S9 frequency (Hz) */

namespace navp::ginan {

std::map<ConstellationEnum, std::map<ObsCodeEnum, FreTypeEnum>> code2Freq = {
    {ConstellationEnum::GPS, {{ObsCodeEnum::NONE, FreTypeEnum::FTYPE_NONE}, {ObsCodeEnum::L1C, FreTypeEnum::F1},
                              {ObsCodeEnum::L1S, FreTypeEnum::F1},          {ObsCodeEnum::L1L, FreTypeEnum::F1},
                              {ObsCodeEnum::L1X, FreTypeEnum::F1},          {ObsCodeEnum::L1P, FreTypeEnum::F1},
                              {ObsCodeEnum::L1W, FreTypeEnum::F1},          {ObsCodeEnum::L1Y, FreTypeEnum::F1},
                              {ObsCodeEnum::L1M, FreTypeEnum::F1},          {ObsCodeEnum::L1N, FreTypeEnum::F1},

                              {ObsCodeEnum::L2C, FreTypeEnum::F2},          {ObsCodeEnum::L2D, FreTypeEnum::F2},
                              {ObsCodeEnum::L2S, FreTypeEnum::F2},          {ObsCodeEnum::L2L, FreTypeEnum::F2},
                              {ObsCodeEnum::L2X, FreTypeEnum::F2},          {ObsCodeEnum::L2P, FreTypeEnum::F2},
                              {ObsCodeEnum::L2W, FreTypeEnum::F2},          {ObsCodeEnum::L2Y, FreTypeEnum::F2},
                              {ObsCodeEnum::L2M, FreTypeEnum::F2},          {ObsCodeEnum::L2N, FreTypeEnum::F2},

                              {ObsCodeEnum::L5I, FreTypeEnum::F5},          {ObsCodeEnum::L5Q, FreTypeEnum::F5},
                              {ObsCodeEnum::L5X, FreTypeEnum::F5}}},

    {ConstellationEnum::GLO,
     {{ObsCodeEnum::NONE, FreTypeEnum::FTYPE_NONE},
      {ObsCodeEnum::L1C, FreTypeEnum::G1},
      {ObsCodeEnum::L1P, FreTypeEnum::G1},

      {ObsCodeEnum::L2C, FreTypeEnum::G2},
      {ObsCodeEnum::L2P, FreTypeEnum::G2},

      {ObsCodeEnum::L3I, FreTypeEnum::G3},
      {ObsCodeEnum::L3Q, FreTypeEnum::G3},
      {ObsCodeEnum::L3X, FreTypeEnum::G3},

      {ObsCodeEnum::L4A, FreTypeEnum::G4},
      {ObsCodeEnum::L4B, FreTypeEnum::G4},
      {ObsCodeEnum::L4X, FreTypeEnum::G4},

      {ObsCodeEnum::L6A, FreTypeEnum::G6},
      {ObsCodeEnum::L6B, FreTypeEnum::G6},
      {ObsCodeEnum::L6X, FreTypeEnum::G6}}},

    {ConstellationEnum::GAL, {{ObsCodeEnum::NONE, FreTypeEnum::FTYPE_NONE}, {ObsCodeEnum::L1A, FreTypeEnum::F1},
                              {ObsCodeEnum::L1B, FreTypeEnum::F1},          {ObsCodeEnum::L1C, FreTypeEnum::F1},
                              {ObsCodeEnum::L1X, FreTypeEnum::F1},          {ObsCodeEnum::L1Z, FreTypeEnum::F1},

                              {ObsCodeEnum::L5I, FreTypeEnum::F5},          {ObsCodeEnum::L5Q, FreTypeEnum::F5},
                              {ObsCodeEnum::L5X, FreTypeEnum::F5},

                              {ObsCodeEnum::L6A, FreTypeEnum::F6},          {ObsCodeEnum::L6B, FreTypeEnum::F6},
                              {ObsCodeEnum::L6C, FreTypeEnum::F6},          {ObsCodeEnum::L6X, FreTypeEnum::F6},
                              {ObsCodeEnum::L6Z, FreTypeEnum::F6},

                              {ObsCodeEnum::L7I, FreTypeEnum::F7},          {ObsCodeEnum::L7Q, FreTypeEnum::F7},
                              {ObsCodeEnum::L7X, FreTypeEnum::F7},

                              {ObsCodeEnum::L8I, FreTypeEnum::F8},          {ObsCodeEnum::L8Q, FreTypeEnum::F8},
                              {ObsCodeEnum::L8X, FreTypeEnum::F8}}},

    {ConstellationEnum::BDS, {{ObsCodeEnum::NONE, FreTypeEnum::FTYPE_NONE}, {ObsCodeEnum::L1D, FreTypeEnum::F1},
                              {ObsCodeEnum::L1P, FreTypeEnum::F1},          {ObsCodeEnum::L1X, FreTypeEnum::F1},
                              {ObsCodeEnum::L1S, FreTypeEnum::F1},          {ObsCodeEnum::L1L, FreTypeEnum::F1},
                              {ObsCodeEnum::L1Z, FreTypeEnum::F1},          {ObsCodeEnum::L1A, FreTypeEnum::F1},
                              {ObsCodeEnum::L1N, FreTypeEnum::F1},

                              {ObsCodeEnum::L2I, FreTypeEnum::B1},          {ObsCodeEnum::L2Q, FreTypeEnum::B1},
                              {ObsCodeEnum::L2X, FreTypeEnum::B1},

                              {ObsCodeEnum::L5D, FreTypeEnum::F5},          {ObsCodeEnum::L5P, FreTypeEnum::F5},
                              {ObsCodeEnum::L5X, FreTypeEnum::F5},

                              {ObsCodeEnum::L6I, FreTypeEnum::B3},          {ObsCodeEnum::L6Q, FreTypeEnum::B3},
                              {ObsCodeEnum::L6X, FreTypeEnum::B3},          {ObsCodeEnum::L6D, FreTypeEnum::B3},
                              {ObsCodeEnum::L6P, FreTypeEnum::B3},          {ObsCodeEnum::L6Z, FreTypeEnum::B3},
                              {ObsCodeEnum::L6A, FreTypeEnum::B3},

                              {ObsCodeEnum::L7I, FreTypeEnum::F7},          {ObsCodeEnum::L7Q, FreTypeEnum::F7},
                              {ObsCodeEnum::L7X, FreTypeEnum::F7},          {ObsCodeEnum::L7D, FreTypeEnum::F7},
                              {ObsCodeEnum::L7P, FreTypeEnum::F7},          {ObsCodeEnum::L7Z, FreTypeEnum::F7},

                              {ObsCodeEnum::L8D, FreTypeEnum::F8},          {ObsCodeEnum::L8P, FreTypeEnum::F8},
                              {ObsCodeEnum::L8Z, FreTypeEnum::F8}}},

    {ConstellationEnum::QZS, {{ObsCodeEnum::NONE, FreTypeEnum::FTYPE_NONE}, {ObsCodeEnum::L1C, FreTypeEnum::F1},
                              {ObsCodeEnum::L1E, FreTypeEnum::F1},          {ObsCodeEnum::L1S, FreTypeEnum::F1},
                              {ObsCodeEnum::L1L, FreTypeEnum::F1},          {ObsCodeEnum::L1X, FreTypeEnum::F1},
                              {ObsCodeEnum::L1Z, FreTypeEnum::F1},          {ObsCodeEnum::L1B, FreTypeEnum::F1},

                              {ObsCodeEnum::L2S, FreTypeEnum::F2},          {ObsCodeEnum::L2L, FreTypeEnum::F2},
                              {ObsCodeEnum::L2X, FreTypeEnum::F2},

                              {ObsCodeEnum::L5I, FreTypeEnum::F5},          {ObsCodeEnum::L5Q, FreTypeEnum::F5},
                              {ObsCodeEnum::L5X, FreTypeEnum::F5},          {ObsCodeEnum::L5D, FreTypeEnum::F5},
                              {ObsCodeEnum::L5P, FreTypeEnum::F5},          {ObsCodeEnum::L5Z, FreTypeEnum::F5},

                              {ObsCodeEnum::L6S, FreTypeEnum::F6},          {ObsCodeEnum::L6L, FreTypeEnum::F6},
                              {ObsCodeEnum::L6X, FreTypeEnum::F6},          {ObsCodeEnum::L6E, FreTypeEnum::F6},
                              {ObsCodeEnum::L6Z, FreTypeEnum::F6}}},

    {ConstellationEnum::IRN,
     {/* NavIC FreTypeEnum::F1 in the works... */

      {ObsCodeEnum::NONE, FreTypeEnum::FTYPE_NONE},
      {ObsCodeEnum::L5A, FreTypeEnum::F5},
      {ObsCodeEnum::L5B, FreTypeEnum::F5},
      {ObsCodeEnum::L5C, FreTypeEnum::F5},
      {ObsCodeEnum::L5X, FreTypeEnum::F5},

      {ObsCodeEnum::L9A, FreTypeEnum::I9},
      {ObsCodeEnum::L9B, FreTypeEnum::I9},
      {ObsCodeEnum::L9C, FreTypeEnum::I9},
      {ObsCodeEnum::L9X, FreTypeEnum::I9}}},

    {ConstellationEnum::SBS,
     {{ObsCodeEnum::NONE, FreTypeEnum::FTYPE_NONE},
      {ObsCodeEnum::L1C, FreTypeEnum::F1},

      {ObsCodeEnum::L5I, FreTypeEnum::F5},
      {ObsCodeEnum::L5Q, FreTypeEnum::F5},
      {ObsCodeEnum::L5X, FreTypeEnum::F5}}},

    {ConstellationEnum::LEO, {{ObsCodeEnum::NONE, FreTypeEnum::FTYPE_NONE}, {ObsCodeEnum::L1C, FreTypeEnum::F1},
                              {ObsCodeEnum::L1S, FreTypeEnum::F1},          {ObsCodeEnum::L1L, FreTypeEnum::F1},
                              {ObsCodeEnum::L1X, FreTypeEnum::F1},          {ObsCodeEnum::L1P, FreTypeEnum::F1},
                              {ObsCodeEnum::L1W, FreTypeEnum::F1},          {ObsCodeEnum::L1Y, FreTypeEnum::F1},
                              {ObsCodeEnum::L1M, FreTypeEnum::F1},          {ObsCodeEnum::L1N, FreTypeEnum::F1},

                              {ObsCodeEnum::L2C, FreTypeEnum::F2},          {ObsCodeEnum::L2D, FreTypeEnum::F2},
                              {ObsCodeEnum::L2S, FreTypeEnum::F2},          {ObsCodeEnum::L2L, FreTypeEnum::F2},
                              {ObsCodeEnum::L2X, FreTypeEnum::F2},          {ObsCodeEnum::L2P, FreTypeEnum::F2},
                              {ObsCodeEnum::L2W, FreTypeEnum::F2},          {ObsCodeEnum::L2Y, FreTypeEnum::F2},
                              {ObsCodeEnum::L2M, FreTypeEnum::F2},          {ObsCodeEnum::L2N, FreTypeEnum::F2},

                              {ObsCodeEnum::L5I, FreTypeEnum::F5},          {ObsCodeEnum::L5Q, FreTypeEnum::F5},
                              {ObsCodeEnum::L5X, FreTypeEnum::F5}}}};

std::map<FreTypeEnum, f64> genericWavelength = {
    {FreTypeEnum::F1, CLIGHT / FREQ1},     {FreTypeEnum::F2, CLIGHT / FREQ2},     {FreTypeEnum::F5, CLIGHT / FREQ5},
    {FreTypeEnum::F6, CLIGHT / FREQ6},     {FreTypeEnum::F7, CLIGHT / FREQ7},     {FreTypeEnum::F8, CLIGHT / FREQ8},
    {FreTypeEnum::B1, CLIGHT / FREQ1_CMP}, {FreTypeEnum::B3, CLIGHT / FREQ3_CMP}, {FreTypeEnum::G1, CLIGHT / FREQ1_GLO},
    {FreTypeEnum::G2, CLIGHT / FREQ2_GLO}, {FreTypeEnum::G3, CLIGHT / FREQ3_GLO}, {FreTypeEnum::G4, CLIGHT / FREQ4_GLO},
    {FreTypeEnum::G6, CLIGHT / FREQ6_GLO}};

std::map<FreTypeEnum, f64> roughFrequency = {
    {FreTypeEnum::F1, FREQ1},     {FreTypeEnum::F2, FREQ1},     {FreTypeEnum::F5, FREQ5},
    {FreTypeEnum::F6, FREQ6},     {FreTypeEnum::F7, FREQ7},     {FreTypeEnum::F8, FREQ8},
    {FreTypeEnum::G1, FREQ1_GLO}, {FreTypeEnum::G2, FREQ2_GLO}, {FreTypeEnum::G3, FREQ3_GLO},
    {FreTypeEnum::G4, FREQ4_GLO}, {FreTypeEnum::G6, FREQ6_GLO}, {FreTypeEnum::B1, FREQ1_CMP},
    {FreTypeEnum::B3, FREQ3_CMP}, {FreTypeEnum::I9, FREQ9_IRN}};

}  // namespace navp::ginan