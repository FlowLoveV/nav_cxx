#pragma once

#include "io/rinex/rinex_record.hpp"
#include "io/stream.hpp"
#include "utils/macro.hpp"

namespace navp::io::rinex {

class RinexStream;

// rinex stream
class NAVP_EXPORT RinexStream : public Stream, public RinexRecord {
 public:
  using Stream::Stream;
  virtual ~RinexStream() override;

 protected:
  virtual void decode_record(Record& record) override;

  virtual void encode_record(Record& record) override;
};

}  // namespace navp::io::rinex