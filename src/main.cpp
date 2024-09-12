
#include <cassert>
#include <print>
#include <vector>

#include "utils/option.hpp"

struct empty {};

struct big {
  int a;
  float b;
  double c;
  std::string d[20];
};
int main(int argc, char* argv[]) {
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

  auto f1 = [](const int& x) { std::println("{}", x); };
  auto m = op1.inspect(f1);

  op1.replace(20);

  op1 = op2;
  auto op3(op1);

  Option<std::vector<int>> x = std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8};
  Option<std::vector<int>> y(std::in_place, {1, 2, 3, 4, 5, 6, 7, 8});
  Option<std::vector<int>> z = None;
  const std::vector<int>& ref_x = x.unwrap();
  std::vector<int> move_x = std::move(x).unwrap();

  auto& ref_z = z.unwrap_or(x.unwrap());
  z = None;
  auto v = z.unwrap_or_else([]() { return std::vector<int>{1, 2, 3}; });
  auto op_double = z | Option<double>(10.0);
  // x.unwarp();
  // z = None;
  // auto vec = z.expected("unwrap failed");
  auto b = y.map([](const std::vector<int>& v) -> Option<size_t> { return Some(v.size()); });
  assert(b == Some(8));
  b = None;
  auto c = b.map_or([](const size_t& s) { return static_cast<double>(s); }, 20.0);
  auto d = b.map_or_else([]() { return 0.0; }, [](const size_t& s) { return static_cast<double>(s); });

  auto e = x.as_ref();
  auto size = e.unwrap().get().size();
  return 0;
}