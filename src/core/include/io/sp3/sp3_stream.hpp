#include "io/stream.hpp"
#include "utils/macro.hpp"

namespace navp::io::sp3 {

inline constexpr auto INVALID_CLOCK_VALUE = 999999.999999;

class Sp3Stream;

class NAVP_EXPORT Sp3Stream : public Fstream {
 public:
  using Fstream::Fstream;

 protected:
  virtual void decode_record(Record& record) override;

  virtual void encode_record(Record& record) override;
};

}  // namespace navp::io::sp3