#pragma once

#include <cpptrace/cpptrace.hpp>
#include <variant>

namespace navp {

// struct Error {
//   unsigned code;
//   std::string message;

//   template <typename T>
//     requires std::is_base_of_v<Error, T>
//   bool operator==(const T& err) noexcept {
//     return code == err.code;
//   }

//   void crash() {
//     nav_error("{}", message);
//     cpptrace::generate_trace(1).print();
//     exit(-1);
//   }
// };

template <typename T, typename E>
class Result;

namespace details {

template <typename T, template <typename...> class Template>
struct is_instance_of : std::false_type {};
template <template <typename...> class Template, typename... Args>
struct is_instance_of<Template<Args...>, Template> : std::true_type {};

};  // namespace details

template <typename T, typename E>
class Result : private std::variant<T, E> {
 private:
  template <typename _Up>
  using __not_self = std::__not_<std::is_same<Result, std::__remove_cvref_t<_Up>>>;
  template <typename... _Cond>
  using _Requires = std::enable_if_t<std::__and_v<_Cond...>, bool>;
  template <typename U>
  using rmcv_ref_t = std::__remove_cvref_t<U>;
  template <typename U>
  using _not = std::__not_<U>;
  template <typename... U>
  using _and = std::__and_<U...>;
  template <typename U>
  using not_result = _not<details::is_instance_of<rmcv_ref_t<U>, Result>>;
  template <typename U>
  using not_variant = _not<details::is_instance_of<rmcv_ref_t<U>, std::variant>>;

  using _Base = std::variant<T, E>;

 public:
  using std::variant<T, E>::variant;
  using std::variant<T, E>::operator=;

  // default constructor(default Ok)
  // only when U can default construct
  // constexpr Result(std::enable_if_t<std::is_default_constructible_v<T>, bool> = true) noexcept(
  //     std::is_nothrow_default_constructible_v<T>) {
  //   this->template emplace<0>();
  // }

  // Result(const Result&) = default;

  // // when T or E is not trivially copy constructible, overload default copy constructor
  // constexpr Result(const Result& _other) noexcept(
  //     _and<std::is_nothrow_copy_constructible<T>, std::is_nothrow_copy_constructible<E>>::value)
  //   requires std::is_copy_constructible_v<T> && std::is_copy_constructible_v<E> &&
  //            (!std::is_trivially_copy_constructible_v<T> || !std::is_trivially_copy_constructible_v<E>)
  // {
  //   if (_other.is_ok()) {
  //     this->template emplace<0>(_other._m_get_ok_value());
  //   } else {
  //     this->template emplace<1>(_other._m_get_err_value());
  //   }
  // }

  // Result(Result&&) = default;

  // // when T or E is not trivially move constructible, overload default move constructor
  // constexpr Result(Result&& _other) noexcept(
  //     _and<std::is_nothrow_move_constructible<T>, std::is_nothrow_move_constructible<E>>::value)
  //   requires std::is_move_constructible_v<T> && std::is_move_constructible_v<E> &&
  //            (!std::is_trivially_move_constructible_v<T> || !std::is_trivially_move_constructible_v<E>)
  // {
  //   if (_other.is_ok()) {
  //     this->template emplace<0>(std::move(_other._m_get_ok_value()));
  //   } else {
  //     this->template emplace<1>(std::move(_other._m_get_err_value()));
  //   }
  // }

  // Result& operator=(const Result&) = default;

  // // when T or E can be move nothrowable, overload copy assign
  // constexpr Result& operator=(const Result& _other) noexcept(
  //     _and<std::is_nothrow_copy_constructible<T>, std::is_nothrow_copy_constructible<E>,
  //          std::is_nothrow_copy_assignable<T>, std::is_nothrow_copy_assignable<E>>::value)
  //   requires std::is_copy_assignable_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<E> &&
  //            std::is_copy_constructible_v<E> &&
  //            (std::is_nothrow_move_constructible_v<T> || std::is_nothrow_move_constructible_v<E>)
  // {
  //   if (_other.is_ok()) {
  //     this->template emplace<0>(_other._m_get_ok_value());
  //   } else {
  //     this->template emplace<1>(_other._m_get_err_value());
  //   }
  // }

  // Result& operator=(Result&&) = default;

  // // when T or E can be move nothrowable, overload move assign
  // constexpr Result& operator=(Result&& _other) noexcept(
  //     _and<std::is_nothrow_move_constructible<T>, std::is_nothrow_move_constructible<E>,
  //          std::is_nothrow_move_assignable<T>, std::is_nothrow_move_assignable<E>>::value)
  //   requires std::is_move_assignable_v<T> && std::is_move_constructible_v<T> && std::is_move_assignable_v<E> &&
  //            std::is_move_constructible_v<E> &&
  //            (std::is_nothrow_move_constructible_v<T> || std::is_nothrow_move_constructible_v<E>)
  // {
  //   if (_other.is_ok()) {
  //     this->template emplace<0>(std::move(_other._m_get_ok_value()));
  //   } else {
  //     this->template emplace<1>(std::move(_other._m_get_err_value()));
  //   }
  // }

  // constexpr ~Result() = default;

  // // construct from U value
  // template <typename U,
  //           _Requires<not_result<U>, not_variant<U>, std::is_constructible<T, U>, std::is_convertible<U, T>> = true>
  // constexpr Result(U&& ok_val) noexcept(std::is_nothrow_constructible_v<T, U>)
  //     : std::variant<T, E>(static_cast<T>(ok_val)) {}

  // template <typename U, _Requires<not_result<U>, not_variant<U>, std::is_constructible<T, U>,
  //                                 _not<std::is_convertible<U, T>>> = false>
  // explicit constexpr Result(U&& ok_val) noexcept(std::is_nothrow_constructible_v<T, U>)
  //     : std::variant<T, E>(static_cast<T>(ok_val)) {}

  // template <typename U,
  //           _Requires<not_result<U>, not_variant<U>, std::is_constructible<E, U>, std::is_convertible<U, E>> = true>
  // constexpr Result(U&& err_val) noexcept(std::is_nothrow_constructible_v<E, U>)
  //     : std::variant<T, E>(static_cast<E>(err_val)) {}

  // template <typename U, _Requires<not_result<U>, not_variant<U>, std::is_constructible<E, U>,
  //                                 _not<std::is_convertible<U, E>>> = false>
  // explicit constexpr Result(U&& err_val) noexcept(std::is_nothrow_constructible_v<E, U>)
  //     : std::variant<T, E>(static_cast<E>(err_val)) {}

  // constrcut from Result<T,E>

  // constrcut in_place

  // template <typename U>
  // constexpr Result(U&& err_val) noexcept {}

  // is_ok,is_err
  constexpr inline bool is_ok() const noexcept { return this->index() == 0; }
  constexpr inline bool is_err() const noexcept { return this->index() == 1; }
  // Err,err
  // constexpr std::optional<T>

  // unwrap
  constexpr T& unwrap() & {
    if (is_ok()) [[likely]] {
      return _m_get_ok_value();
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
    }
  }
  constexpr const T& unwrap() const& {
    if (is_ok()) [[likely]] {
      return std::get<T>(*this);
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
    }
  }
  constexpr const T&& unwrap() const&& {
    if (is_ok()) [[likely]] {
      return std::move(std::get<T>(*this));
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
    }
  }
  constexpr T&& unwrap() && {
    if (is_ok()) [[likely]] {
      return std::move(std::get<T>(*this));
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
    }
  }

  // unwrap_or
  template <typename U>
  constexpr T unwrap_or(U&& _default) const& noexcept(
      std::__and_v<std::is_nothrow_copy_constructible<T>, std::is_nothrow_convertible<U, T>>) {
    static_assert(std::is_copy_constructible_v<T>);
    static_assert(std::is_convertible_v<U, T>);
    return is_ok() ? _m_get_ok_value() : static_cast<T>(std::forward<U>(_default));
  }
  template <typename U>
  constexpr T unwrap_or(U&& _default) && noexcept(
      std::__and_v<std::is_nothrow_move_constructible<T>, std::is_nothrow_convertible<U, T>>) {
    static_assert(std::is_copy_constructible_v<T>);
    static_assert(std::is_convertible_v<U, T>);
    return is_ok() ? std::move(_m_get_ok_value()) : static_cast<T>(std::forward<U>(_default));
  }

  // unwarp_or_else

  // is_ok

  // is_ok_and

  // is_err

  // is_err_and

  // and_then

 protected:
  // uncecked get ok value
  constexpr inline T& _m_get_ok_value() & { return std::get<T>(*this); }
  constexpr inline const T& _m_get_ok_value() const& { return std::get<T>(*this); }
  constexpr inline T&& _m_get_ok_value() && { return std::get<T>(*this); }
  constexpr inline const T&& _m_get_ok_value() const&& { return std::get<T>(*this); }
  // uncecked get err value
  constexpr inline T& _m_get_err_value() & { return std::get<E>(*this); }
  constexpr inline const T& _m_get_err_value() const& { return std::get<E>(*this); }
  constexpr inline T&& _m_get_err_value() && { return std::get<E>(*this); }
  constexpr inline const T&& _m_get_err_value() const&& { return std::get<E>(*this); }
};

template <typename T, typename E>
constexpr Result<T, E> Ok(const T& ok_val) noexcept {
  return Result<T, E>(ok_val);
}

template <typename T, typename E>
constexpr Result<T, E> Ok(T&& ok_val) noexcept {
  return Result<T, E>(std::move(ok_val));
}

template <typename T, typename E, typename... Args>
  requires std::is_constructible_v<T, Args...>
constexpr Result<T, E> Ok(Args... args) noexcept(std::is_nothrow_constructible_v<T, Args...>) {
  return Result<T, E>(std::forward<Args>(args)...);
}

template <typename T, typename E, typename U, typename... Args>
  requires std::is_constructible_v<T, std::initializer_list<U>&, Args...>
constexpr Result<T, E> Ok(std::initializer_list<U> list, Args... args) noexcept(
    std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>) {
  return Result<T, E>(list, std::forward<Args>(args)...);
}

template <typename T, typename E>
constexpr Result<T, E> Err(const E& err_val) noexcept {
  return Result<T, E>(err_val);
}

template <typename T, typename E>
constexpr Result<T, E> Err(E&& err_val) noexcept {
  return Result<T, E>(std::move(err_val));
}

template <typename T, typename E, typename... Args>
  requires std::is_constructible_v<E, Args...>
constexpr Result<T, E> Err(Args... args) noexcept(std::is_nothrow_constructible_v<E, Args...>) {
  return Result<T, E>(std::forward<Args>(args)...);
}

template <typename T, typename E, typename U, typename... Args>
  requires std::is_constructible_v<E, std::initializer_list<U>&, Args...>
constexpr Result<T, E> Err(std::initializer_list<U> list, Args... args) noexcept(
    std::is_nothrow_constructible_v<E, std::initializer_list<U>&, Args...>) {
  return Result<T, E>(list, std::forward<Args>(args)...);
}

}  // namespace navp