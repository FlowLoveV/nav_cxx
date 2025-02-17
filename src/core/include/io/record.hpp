#pragma once

#include "utils/macro.hpp"

namespace navp::io {

// forward declaration
class Fstream;

class Record;

class NAVP_EXPORT Record {
 public:
  virtual ~Record();

  // send a "record" to the given stream
  void put_record(Fstream& stream) const;

  // get a "record" from the given stream
  void get_record(Fstream& s);

 protected:
  virtual void really_get_record(Fstream& s);

  virtual void really_put_record(Fstream& s) const;
};

}  // namespace navp::io