#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <Eigen/Eigen>
#include <expected>

#include "doctest.h"
#include "utils/option.hpp"
#include "utils/result.hpp"

using navp::Err;
using navp::Ok;
using navp::Result;

TEST_CASE("Triviality") {
  typedef Ok<int> ok_t;
  typedef Err<int> err_t;
  typedef Result<int, int> result_t;
  typedef std::expected<int, int> expected_t;

  static_assert(!std::is_trivially_constructible_v<ok_t>);
  static_assert(std::is_trivially_copy_constructible<ok_t>::value);
  static_assert(std::is_trivially_copy_assignable<ok_t>::value);
  static_assert(std::is_trivially_move_constructible<ok_t>::value);
  static_assert(std::is_trivially_move_assignable<ok_t>::value);
  static_assert(std::is_trivially_destructible<ok_t>::value);

  static_assert(!std::is_trivially_constructible_v<err_t>);
  static_assert(std::is_trivially_copy_constructible<err_t>::value);
  static_assert(std::is_trivially_copy_assignable<err_t>::value);
  static_assert(std::is_trivially_move_constructible<err_t>::value);
  static_assert(std::is_trivially_move_assignable<err_t>::value);
  static_assert(std::is_trivially_destructible<err_t>::value);

  static_assert(!std::is_trivially_constructible_v<result_t>);
  static_assert(std::is_trivially_copy_constructible<result_t>::value);
  static_assert(!std::is_trivially_copy_assignable<result_t>::value);
  static_assert(std::is_trivially_move_constructible<result_t>::value);
  static_assert(!std::is_trivially_move_assignable<result_t>::value);
  static_assert(std::is_trivially_destructible<result_t>::value);

  static_assert(!std::is_trivially_constructible_v<expected_t>);
  static_assert(std::is_trivially_copy_constructible<expected_t>::value);
  static_assert(!std::is_trivially_copy_assignable<expected_t>::value);
  static_assert(std::is_trivially_move_constructible<expected_t>::value);
  static_assert(!std::is_trivially_move_assignable<expected_t>::value);
  static_assert(std::is_trivially_destructible<expected_t>::value);

  {
    struct T {
      T(const T&) = default;
      T(T&&) = default;
      T& operator=(const T&) = default;
      T& operator=(T&&) = default;
      ~T() = default;
    };
    static_assert(std::is_trivially_copy_constructible<Ok<T>>::value);
    static_assert(std::is_trivially_copy_assignable<Ok<T>>::value);
    static_assert(std::is_trivially_move_constructible<Ok<T>>::value);
    static_assert(std::is_trivially_move_assignable<Ok<T>>::value);
    static_assert(std::is_trivially_destructible<Ok<T>>::value);

    static_assert(std::is_trivially_copy_constructible<Err<T>>::value);
    static_assert(std::is_trivially_copy_assignable<Err<T>>::value);
    static_assert(std::is_trivially_move_constructible<Err<T>>::value);
    static_assert(std::is_trivially_move_assignable<Err<T>>::value);
    static_assert(std::is_trivially_destructible<Err<T>>::value);

    static_assert(!std::is_trivially_constructible_v<Result<T, T>>);
    static_assert(std::is_trivially_copy_constructible<Result<T, T>>::value);
    static_assert(!std::is_trivially_copy_assignable<Result<T, T>>::value);
    static_assert(std::is_trivially_move_constructible<Result<T, T>>::value);
    static_assert(!std::is_trivially_move_assignable<Result<T, T>>::value);
    static_assert(std::is_trivially_destructible<Result<T, T>>::value);
  }

  {
    struct T {
      T(const T&) {}
      T(T&&){};
      T& operator=(const T&) { return *this; }
      T& operator=(T&&) { return *this; };
      ~T() {}
    };
    static_assert(!std::is_trivially_copy_constructible<Ok<T>>::value);
    static_assert(!std::is_trivially_copy_assignable<Ok<T>>::value);
    static_assert(!std::is_trivially_move_constructible<Ok<T>>::value);
    static_assert(!std::is_trivially_move_assignable<Ok<T>>::value);
    static_assert(!std::is_trivially_destructible<Ok<T>>::value);

    static_assert(!std::is_trivially_copy_constructible<Err<T>>::value);
    static_assert(!std::is_trivially_copy_assignable<Err<T>>::value);
    static_assert(!std::is_trivially_move_constructible<Err<T>>::value);
    static_assert(!std::is_trivially_move_assignable<Err<T>>::value);
    static_assert(!std::is_trivially_destructible<Err<T>>::value);

    static_assert(!std::is_trivially_constructible_v<Result<T, T>>);
    static_assert(!std::is_trivially_copy_constructible<Result<T, T>>::value);
    static_assert(!std::is_trivially_copy_assignable<Result<T, T>>::value);
    static_assert(!std::is_trivially_move_constructible<Result<T, T>>::value);
    static_assert(!std::is_trivially_move_assignable<Result<T, T>>::value);
    static_assert(!std::is_trivially_destructible<Result<T, T>>::value);
  }
}

TEST_CASE("Constructor,Assignment for Ok and Err") {
  // Here we only test the ok type and not the err type, because their implementations are the same

  // from value (No implicit conversion)
  Ok<int> ok1 = 10;
  Ok<int> ok2(10);
  Ok<int> ok3 = Ok(10);

  // from value (implicit conversion)
  Ok<short> ok4 = 10;
  Ok<short> ok5(10);
  Ok<short> ok6 = Ok(10);

  // constrcut in_place
  typedef Ok<std::vector<int>> T;
  T t1({1, 2, 3, 4, 5});
  T t2 = T({1, 2, 3, 4, 5, 6});

  // assign from Ok (No implicit conversion)
  ok1 = Ok(20);
  ok1 = ok2;
  // assign from Ok (implicit conversion)
  ok1 = ok4;
  // assign from value (No implicit conversion)
  ok1 = 20;
  // assign from value (implicit conversion)
  ok4 = 20;

  Err<std::string> err1("error!");
}

TEST_CASE("Constructor,Assignment for Result") {
  auto ok = Ok("Done!");
  auto err = Err("fail!");
  typedef Result<std::string, std::string> T;
  T t0 = ok;
  CHECK(t0.is_ok());
  t0 = err;
  CHECK(t0.is_err());

  T t1 = err;
  CHECK(t1.is_err());
  t1 = ok;
  CHECK(t1.is_ok());

  T t2 = Ok("Done!");
  CHECK(t2.is_ok());
  T t3 = Err("Error");
  CHECK(t3.is_err());

  typedef Result<double, std::string> U;
  U u0(10);
  CHECK(u0.unwrap() == 10);
  U u1 = 10;
  CHECK(u1.unwrap() == 10);
  U u2 = "Error";
  CHECK(u2.is_err());
  CHECK(u2.unwrap_err() == "Error");
}

TEST_CASE("Operator") {
  typedef Result<double, std::string> U;
  U u0 = 20.0;
  U u1 = "Error";
  CHECK(u0.is_ok_and([](const double& val) { return val >= 10; }));
  CHECK(u1.is_err_and([](const std::string& err) { return err == "Error"; }));

  auto u2 = u0 | u1;
  CHECK(u2.is_err());
  CHECK(u2.unwrap_err() == "Error");

  U u3 = 194910011400.0;
  auto u4 = u0 | u3;
  CHECK(u4.is_ok());
  CHECK(u4.unwrap() == 194910011400.0);
}

TEST_CASE("Extension") {
  typedef Result<double, std::string> U;

  U u1 = 19760909.0;
  auto u2 = u1.and_then<bool>([](const double& val) -> Result<bool, std::string> { return val >= 0.0; });
  CHECK(u2.is_ok());

  U u3 = "Goodbye Rust!";
  CHECK(std::move(u3).ok().is_none());
  U u4 = "Hello Modern C++!";
  CHECK(std::move(u4).err().is_some());

  U u5 = "China";
  CHECK_THROWS(u5.expect("try to unwrap a result with err val"));
  U u6 = 2000;
  CHECK_THROWS(u6.expect_err("try to unwrap_err a result with ok val"));

  U u7 = 3000;
  u7.inspect([](const double& val) {
    if (val <= 2000.0) {
      // do something
    }
  });

  U u8 = "Error";
  u8.inspect_err([](const std::string& str) {
    if (str == "Error") {
      // do something
    }
  });

  U u9 = "Exit";
  auto ok1 = u9.unwrap_or(200.0);
  CHECK(ok1 == 200.0);

  auto u10 = std::move(u9).unwrap_or_default();
  CHECK(u10 == 0.0);

  U u11 = "American";
  auto ok3 = std::move(u11).unwrap_or_else([](const std::string& err) {
    if (err == "American") {
      return 1949.0;
    }
    return 2024.0;
  });
  CHECK(ok3 == 1949.0);

  U u12 = 1314;
  auto x1 = u12.map<bool>([](const double& val) -> bool { return val >= 10; });
  CHECK(x1.unwrap() == true);
  auto x2 = u12.map_err<std::string>([](const std::string& val) { return val.substr(0); });
  CHECK(x2.unwrap() == 1314);

  U u13 = 20;
  auto x3 = u13.map_or_else<bool>([](const std::string& str) { return str == "Error"; },
                                  [](const double& val) { return val >= 0.0; });
  CHECK(x3 == true);
}

TEST_CASE("Ambiguous") {
  typedef std::runtime_error exception;
  typedef Eigen::Matrix3<double> Matrix;
  typedef Result<Matrix, exception> Result_t;
  Result_t result1 = exception("dsaa");
}