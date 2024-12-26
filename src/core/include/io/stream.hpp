#pragma once

#include <fstream>

#include "utils/logger.hpp"
#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace navp::io {

// forward declaration
class Record;

class Fstream;

class NAVP_EXPORT Fstream : public std::fstream {
 public:
  Fstream();

  virtual ~Fstream();

  Fstream(std::string_view filename, std::ios::openmode mode, std::shared_ptr<spdlog::logger> logger = nullptr);

  virtual void open(std::string_view filename, std::ios::openmode mode);

  std::string filename;
  i32 record_number;

  friend class Record;

 protected:
  void reset(std::string_view filename, std::ios::openmode mode);

  virtual void decode_record(Record& record) = 0;

  virtual void encode_record(Record& record) = 0;

  std::shared_ptr<spdlog::logger> logger_;
};

template <typename Derived>
concept nav_stream_type = std::is_base_of_v<Fstream, Derived>;

}  // namespace navp::io