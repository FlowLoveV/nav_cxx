#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <vector>

#include "rtklib.h"
#include "types.hpp"

namespace nav::sensor {

class Record;

class Record {
 public:
  virtual ~Record() = default;

 protected:
  virtual int from_rinex(const char* path);
  virtual int from_rtcm(const char* path);
};

class GnssObs : public Record {
 public:
  virtual int from_rinex(const char* path) final;
  virtual int from_rtcm(const char* path) final;

 private:
  obs_t _obs;
};

template <typename Src, typename Dst>
class RecordConverter {
 public:
  typedef Src SrcType;
  typedef Dst DstType;

  static void transform(const Src* const src, Dst& dst);

  static Dst into(const Src* const src) {
    DstType dst{};
    RecordConverter<Src, Dst>::transform(src, dst);
    return dst;
  }
};

// gnss record
class GnssRecord {
 public:
  struct Record {
    f32 doppler, snr;
    f64 phase, pseudorange;
    LliFlagEnum lli;
    Carrier carrier;
  };
  typedef std::map<gtime_t, std::map<Sv, std::tuple<EpochFlagEnum, std::vector<Record>>>> RecordType;

 private:
  RecordType record_;
};

class LocationRecord {
 public:
  typedef Eigen::Vector3d RecordType;

  RecordType position;
  RecordType velocity;
};

}  // namespace nav::sensor
