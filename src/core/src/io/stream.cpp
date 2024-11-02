#include "io/stream.hpp"

// #include <string>

namespace navp::io {

Stream::Stream() : record_number(0) {}

Stream::~Stream() = default;

void Stream::reset(const char* _filename, std::ios::openmode mode) {
  close();
  clear();
  filename = std::string(_filename);
  record_number = 0;
  std::fstream::open(_filename, mode);
}

void Stream::open(const char* _filename, std::ios::openmode mode) { reset(_filename, mode); }

void Stream::open(const std::string& _filename, std::ios::openmode mode) { reset(_filename.c_str(), mode); }

Stream::Stream(const char* _filename, std::ios::openmode mode) { reset(_filename, mode); }

Stream::Stream(const std::string& _filename, std::ios::openmode mode) { reset(_filename.c_str(), mode); }


}  // namespace navp::io