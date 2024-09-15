#include "utils/option.hpp"

using namespace navp;

int main() {
  struct item {};
  struct some {
    explicit some(const item& _item){};
  };

  item i1;
  some s1( std::move(i1));
  return 0;
}