#pragma once

#include "utils/macro.hpp"

namespace navp::io {

// forward declaration
class Stream;

class Record;

class NAVP_EXPORT Record {
 public:
  virtual ~Record();

  // send a "record" to the given stream
  void put_record(Stream& stream);

  // get a "record" from the given stream
  void get_record(Stream& s);

 protected:
  virtual void really_get_record(Stream& s);

  virtual void really_put_record(Stream& s);
};

}  // namespace navp::io