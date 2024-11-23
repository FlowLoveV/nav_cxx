#pragma once

#include <fstream>

#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace navp::io {

// forward declaration
class Record;

class Stream;

class NAVP_EXPORT Stream : public std::fstream {
 public:
  Stream();

  virtual ~Stream();

  Stream(std::string_view filename, std::ios::openmode mode = std::ios::in);

  virtual void open(std::string_view filename, std::ios::openmode mode);

  std::string filename;
  i32 record_number;

  friend class Record;

 protected:
  void reset(std::string_view filename, std::ios::openmode mode);

  virtual void decode_record(Record& record) = 0;

  virtual void encode_record(Record& record) = 0;
};

}  // namespace navp::io