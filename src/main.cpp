#include <iostream>
#include <toml++/toml.hpp>

using namespace std::literals;

int main() {
  int a = 10;
  bool b = ((a = 20) > 10);
  std::cout << b;
}
