#pragma once

#include <expected>
#include <string>

namespace navp {

struct Error {
  std::string message;
};

template <typename T>
struct Result : public std::expected<T, Error> {
  
};

}  // namespace navp