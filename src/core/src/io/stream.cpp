#include "io/stream.hpp"

// #include <string>

namespace navp::io {

Stream::Stream() : record_number(0) {}

Stream::~Stream() = default;

void Stream::reset(std::string_view _filename, std::ios::openmode mode) {
  close();
  clear();
  filename = std::string(_filename);
  record_number = 0;
  std::fstream::open(_filename.data(), mode);
}

void Stream::open(std::string_view _filename, std::ios::openmode mode) { reset(_filename, mode); }

Stream::Stream(std::string_view _filename, std::ios::openmode mode, std::shared_ptr<spdlog::logger> logger)
    : logger_(logger) {
  reset(_filename, mode);
}

}  // namespace navp::io