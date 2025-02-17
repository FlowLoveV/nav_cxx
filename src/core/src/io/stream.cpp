#include "io/stream.hpp"

#include <filesystem>

namespace navp::io {

Fstream::Fstream() : record_number(0) {}

Fstream::~Fstream() = default;

void Fstream::reset(std::string_view _filename, std::ios::openmode mode) {
  close();
  clear();
  std::fstream::open(_filename.data(), mode);
  auto path = std::filesystem::path(_filename);
  filename = std::filesystem::canonical(path);
  record_number = 0;
}

void Fstream::open(std::string_view _filename, std::ios::openmode mode) { reset(_filename, mode); }

Fstream::Fstream(std::string_view _filename, std::ios::openmode mode, std::shared_ptr<spdlog::logger> logger)
    : logger_(logger) {
  reset(_filename, mode);
}

void Fstream::new_line() noexcept { *this << '\n'; }

}  // namespace navp::io