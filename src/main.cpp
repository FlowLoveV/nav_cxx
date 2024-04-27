#include <chrono>
#include <print>

int main(int argc, char** argv) {
  using namespace std::chrono_literals;
  std::chrono::duration<double_t, std::ratio<1, 1000000>> nanos1{
      0.123456789123456789s};
  std::chrono::duration<double_t, std::ratio<1, 1000000>> nanos2{604800s};
  auto nanos = nanos1 + nanos2;
  auto n = nanos - nanos2;
  std::print("{}", n.count());
  return 0;
}