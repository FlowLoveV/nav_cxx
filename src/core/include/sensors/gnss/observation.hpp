#pragma once

#include "space_time/time.hpp"
#include "types.hpp"

namespace navp::sensor::gnss {

class RecordGnssObs;

/// `gnss record`
class RecordGnssObs {
 public:
  struct Record {
    f32 doppler, snr;
    f64 phase, pseudorange;
    LliFlagEnum lli;
    Carrier carrier;
  };
  typedef std::map<Epoch<UTC>, std::map<Sv, std::tuple<EpochFlagEnum, std::vector<Record>>>>
      RecordType;

 private:
  RecordType record_;
};

}  // namespace navp::sensor::gnss