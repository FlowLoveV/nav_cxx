
#include <cassert>
#include <print>

#include "utils/option.hpp"

struct empty {};

struct big {
  int a;
  float b;
  double c;
  std::string d[20];
};
int main(int argc, char *argv[]) {
  using namespace navp;
  Option<int> op1 = Some(10);
  Option<int> op2 = None;
  std::println("{}", sizeof(empty));
  std::println("{}", sizeof(std::in_place_t));
  assert(op1.is_some());
  assert(!op2.is_some());

  auto f = [](int x) { return x > 5; };
  assert(op1.is_some_and(f));
  assert(op2.is_none_or(f));

  op1.insert(5);
  op2.insert(20);
  assert(op1 == Some(5));
  assert(op2 == Some(20));

  auto f1 = [](const int& x) {std::println("{}",x);};
  auto m = op1.inspect(f1);
  return 0;
}