#pragma once

#include "io/rinex/rinex_record.hpp"
#include "io/stream.hpp"
#include "utils/macro.hpp"

// forward declaration
namespace navp::sensors::gnss {
class GnssObsRecord;
}

namespace navp::io::rinex {

class RinexStream;

// rinex stream
class NAVP_EXPORT RinexStream : public Fstream, public RinexRecord {
 public:
  using Fstream::Fstream;
  virtual ~RinexStream() override;

  void decode_header(Record& record);

  void decode_body(Record& record);

 protected:
  virtual void decode_record(Record& record) override;

  virtual void encode_record(const Record& record) override;
};

}  // namespace navp::io::rinex