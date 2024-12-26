#include "io/stream.hpp"

#include <filesystem>

// #include <string>

namespace navp::io {

Fstream::Fstream() : record_number(0) {}

Fstream::~Fstream() = default;

void Fstream::reset(std::string_view _filename, std::ios::openmode mode) {
  close();
  clear();
  filename = std::filesystem::canonical(_filename);
  record_number = 0;
  std::fstream::open(_filename.data(), mode);
}

void Fstream::open(std::string_view _filename, std::ios::openmode mode) { reset(_filename, mode); }

Fstream::Fstream(std::string_view _filename, std::ios::openmode mode, std::shared_ptr<spdlog::logger> logger)
    : logger_(logger) {
  reset(_filename, mode);
}

}  // namespace navp::io