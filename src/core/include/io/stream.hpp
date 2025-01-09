#pragma once

#include <spdlog/spdlog.h>

#include <fstream>

#include "utils/macro.hpp"
#include "utils/types.hpp"

namespace spdlog {
class logger;
}

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

  template <typename... Args>
  void log(Args... args) {
    logger_->log(std::forward<Args>(args)...);
  }

  template <typename... Args>
  void trace(Args... args) {
    logger_->trace(std::forward<Args>(args)...);
  }

  template <typename... Args>
  void debug(Args... args) {
    logger_->debug(std::forward<Args>(args)...);
  }

  template <typename... Args>
  void info(Args... args) {
    logger_->info(std::forward<Args>(args)...);
  }

  template <typename... Args>
  void warn(Args... args) {
    logger_->warn(std::forward<Args>(args)...);
  }

  template <typename... Args>
  void error(Args... args) {
    logger_->error(std::forward<Args>(args)...);
  }

  template <typename... Args>
  void critical(Args... args) {
    logger_->critical(std::forward<Args>(args)...);
  }

  std::string filename;
  i32 record_number;

  friend class Record;

  static constexpr char AnnotationSymbols = '%';

 protected:
  void reset(std::string_view filename, std::ios::openmode mode);

  virtual void decode_record(Record& record) = 0;

  virtual void encode_record(Record& record) = 0;

  std::shared_ptr<spdlog::logger> logger_;
};

template <typename Derived>
concept nav_fstream_type = std::is_base_of_v<Fstream, Derived>;

}  // namespace navp::io