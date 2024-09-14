#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <expected>

#include "doctest.h"
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

  struct error1 {
    std::string msg;
  };
  struct error2 {
    std::string msg;
  };
  typedef Result<double, error1> R1;
  typedef Result<double, error2> R2;
  R1 r1_1 = 20;
  R2 r2_1 = "Error!";
}