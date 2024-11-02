#include <print>

#include "io/stream.hpp"

int main() {
  std::println("sizeof std::fstream is {}", sizeof(std::fstream));
  std::println("sizeof Stream is {}", sizeof(navp::io::Stream));
  return 0;
}