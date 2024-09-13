#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "utils/result.hpp"

using navp::Result;
using navp::Ok;
using navp::Err;

TEST_CASE("constructor") { 
    typedef Result<int, std::string> T;
    auto t0 = Ok<int,std::string>(10);
    T t1 = 10;
    T t2 = "Hello World!";
    t1 = t2;
    // t1 = Ok(10);
}