#pragma once

#include <cpptrace/cpptrace.hpp>
#include <variant>

#include "template_utils.hpp"

namespace navp {

class result_error : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

template <typename T>
class Option;

template <typename T, typename E>
class Result;

template <typename T>
struct Ok;

template <typename E>
struct Err;

namespace details {

struct ok_tag_t {};
struct err_tag_t {};

// implmet Ok and Err
template <typename T, typename... TagArgs>
struct result_construct_assgin_base {
  T val;
  result_construct_assgin_base() = default;

  constexpr result_construct_assgin_base() noexcept(std::is_nothrow_default_constructible_v<T>)
    requires std::is_default_constructible_v<T>
      : val() {}

  // from value
  template <typename U>
  constexpr result_construct_assgin_base(U&& val) noexcept(std::is_nothrow_constructible_v<T, U>)
    requires std::is_constructible_v<T, U> && std::is_convertible_v<U, T>
      : val(std::forward<U>(val)) {}

  template <typename U>
  explicit constexpr result_construct_assgin_base(U&& val) noexcept(std::is_nothrow_constructible_v<T, U>)
    requires std::is_constructible_v<T, U> && (!std::is_convertible_v<U, T>)
      : val(std::forward<U>(val)) {}

  // from result_construct_assgin_base<U>
  template <typename U>
  constexpr result_construct_assgin_base(const result_construct_assgin_base<U, TagArgs...>& other) noexcept(
      std::is_nothrow_constructible_v<T, const U&>)
    requires(!std::is_same_v<U, T>) && std::is_constructible_v<T, const U&> && std::is_convertible_v<const U&, T>
      : val(other.val) {}

  template <typename U>
  explicit constexpr result_construct_assgin_base(const result_construct_assgin_base<U, TagArgs...>& other) noexcept(
      std::is_nothrow_constructible_v<T, const U&>)
    requires(!std::is_same_v<U, T>) && std::is_constructible_v<T, const U&> && (!std::is_convertible_v<const U&, T>)
      : val(std::move(other.val)) {}

  template <typename U>
  constexpr result_construct_assgin_base(result_construct_assgin_base<U, TagArgs...>&& other) noexcept(
      std::is_nothrow_constructible_v<T, const U&>)
    requires(!std::is_same_v<U, T>) && std::is_constructible_v<T, U> && std::is_convertible_v<U, T>
      : val(std::move(other.val)) {}

  template <typename U>
  explicit constexpr result_construct_assgin_base(result_construct_assgin_base<U, TagArgs...>&& other) noexcept(
      std::is_nothrow_constructible_v<T, const U&>)
    requires(!std::is_same_v<U, T>) && std::is_constructible_v<T, U> && (!std::is_convertible_v<U, T>)
      : val(std::move(other.val)) {}

  template <typename U>
  constexpr result_construct_assgin_base(result_construct_assgin_base<U, TagArgs...>&& other) noexcept {}

  template <typename... Args>
  constexpr result_construct_assgin_base(Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>)
    requires std::is_constructible_v<T, Args...>
      : val(std::forward<Args>(args)...) {}

  template <typename U, typename... Args>
  constexpr result_construct_assgin_base(std::initializer_list<U> list, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>)
    requires std::is_constructible_v<T, std::initializer_list<U>&, Args...>
      : val(list, std::forward<Args>(args)...) {}

  template <typename U>
  constexpr result_construct_assgin_base& operator=(U&& val) noexcept(std::is_nothrow_constructible_v<T, U> &&
                                                                      std::is_nothrow_assignable_v<T, U>)
    requires std::is_constructible_v<T, U> && std::is_assignable_v<T, U>
  {
    this->val = std::forward<U>(val);
    return *this;
  }

  template <typename U>
  constexpr result_construct_assgin_base& operator=(const result_construct_assgin_base<U, TagArgs...>& rhs) noexcept(
      std::is_nothrow_constructible_v<T, const U&> && std::is_nothrow_assignable_v<T, const U&>)
    requires std::is_constructible_v<T, const U&> && std::is_assignable_v<T, const U&>
  {
    this->val = std::forward<U>(val);
    return *this;
  }

  template <typename U>
  constexpr result_construct_assgin_base& operator=(result_construct_assgin_base<U, TagArgs...>&& rhs) noexcept(
      std::is_nothrow_constructible_v<T, U> && std::is_nothrow_assignable_v<T, U>)
    requires std::is_constructible_v<T, U> && std::is_assignable_v<T, U>
  {
    this->val = std::forward<U>(val);
    return *this;
  }
};

};  // namespace details

template <typename T>
struct Ok : public details::result_construct_assgin_base<T, details::ok_tag_t> {
  using details::result_construct_assgin_base<T, details::ok_tag_t>::result_construct_assgin_base;
};
template <typename E>
struct Err : public details::result_construct_assgin_base<E, details::err_tag_t> {
  using details::result_construct_assgin_base<E, details::err_tag_t>::result_construct_assgin_base;
};

// decuce helper
template <typename T>
Ok(T) -> Ok<T>;
template <typename E>
Err(E) -> Err<E>;

template <typename T, typename E>
class Result : private std::variant<Ok<T>, Err<E>> {
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

  using _Base = std::variant<Ok<T>, Err<E>>;
  using ok_t = Ok<T>;
  using err_t = Err<E>;

 public:
  // default constructor(default Ok)
  // only when U can default construct
  constexpr Result() noexcept(std::is_nothrow_default_constructible_v<T>)
    requires std::is_default_constructible_v<T>
      : _Base(T()) {}

  Result(const Result&) = default;

  // when T or E is not trivially copy constructible, overload default copy constructor
  constexpr Result(const Result& _other) noexcept(
      _and<std::is_nothrow_copy_constructible<T>, std::is_nothrow_copy_constructible<E>>::value)
    requires std::is_copy_constructible_v<T> && std::is_copy_constructible_v<E> &&
             (!std::is_trivially_copy_constructible_v<T> || !std::is_trivially_copy_constructible_v<E>)
  {
    if (_other.is_ok()) {
      this->template emplace<0>(_other._m_get_ok_value());
    } else {
      this->template emplace<1>(_other._m_get_err_value());
    }
  }

  Result(Result&&) = default;

  // when T or E is not trivially move constructible, overload default move constructor
  constexpr Result(Result&& _other) noexcept(
      _and<std::is_nothrow_move_constructible<T>, std::is_nothrow_move_constructible<E>>::value)
    requires std::is_move_constructible_v<T> && std::is_move_constructible_v<E> &&
             (!std::is_trivially_move_constructible_v<T> || !std::is_trivially_move_constructible_v<E>)
  {
    if (_other.is_ok()) {
      this->template emplace<0>(std::move(_other._m_get_ok_value()));
    } else {
      this->template emplace<1>(std::move(_other._m_get_err_value()));
    }
  }

  Result& operator=(const Result&) = delete;

  // when T or E can be move nothrowable, overload copy assign
  constexpr Result& operator=(const Result& _other) noexcept(
      _and<std::is_nothrow_copy_constructible<T>, std::is_nothrow_copy_constructible<E>,
           std::is_nothrow_copy_assignable<T>, std::is_nothrow_copy_assignable<E>>::value)
    requires std::is_copy_assignable_v<T> && std::is_copy_constructible_v<T> && std::is_copy_assignable_v<E> &&
             std::is_copy_constructible_v<E> &&
             (std::is_nothrow_move_constructible_v<T> || std::is_nothrow_move_constructible_v<E>)
  {
    if (_other.is_ok()) {
      this->template emplace<0>(_other._m_get_ok_value());
    } else {
      this->template emplace<1>(_other._m_get_err_value());
    }
  }

  Result& operator=(Result&&) = default;

  // when T or E can be move nothrowable, overload move assign
  constexpr Result& operator=(Result&& _other) noexcept(
      _and<std::is_nothrow_move_constructible<T>, std::is_nothrow_move_constructible<E>,
           std::is_nothrow_move_assignable<T>, std::is_nothrow_move_assignable<E>>::value)
    requires std::is_move_assignable_v<T> && std::is_move_constructible_v<T> && std::is_move_assignable_v<E> &&
             std::is_move_constructible_v<E> &&
             (std::is_nothrow_move_constructible_v<T> || std::is_nothrow_move_constructible_v<E>)
  {
    if (_other.is_ok()) {
      this->template emplace<0>(std::move(_other._m_get_ok_value()));
    } else {
      this->template emplace<1>(std::move(_other._m_get_err_value()));
    }
  }

  constexpr ~Result() = default;

  // construct from ok_t or err_t
  template <typename U, _Requires<not_result<U>, not_variant<U>, std::is_constructible<ok_t, U>,
                                  std::is_convertible<U, ok_t>> = true>
  constexpr Result(U&& ok_val) noexcept(std::is_nothrow_constructible_v<T, U>) : _Base(std::forward<U>(ok_val)) {}

  template <typename U, _Requires<not_result<U>, not_variant<U>, std::is_constructible<ok_t, U>,
                                  _not<std::is_same<ok_t, err_t>>, _not<std::is_convertible<U, ok_t>>> = false>
  explicit constexpr Result(U&& ok_val) noexcept(std::is_nothrow_constructible_v<ok_t, U>)
      : _Base(std::forward<U>(ok_val)) {}

  template <typename G, _Requires<not_result<G>, not_variant<G>, std::is_constructible<err_t, G>,
                                  std::is_convertible<G, err_t>> = true>
  constexpr Result(G&& err_val) noexcept(std::is_nothrow_constructible_v<err_t, G>) : _Base(std::forward<G>(err_val)) {}

  template <typename G, _Requires<not_result<G>, not_variant<G>, std::is_constructible<err_t, G>,
                                  _not<std::is_same<ok_t, err_t>>, _not<std::is_convertible<G, err_t>>> = false>
  explicit constexpr Result(G&& err_val) noexcept(std::is_nothrow_constructible_v<err_t, G>)
      : _Base(std::forward<G>(err_val)) {}

  // assign operator from Ok<U>,Err<G>
  template <typename U, _Requires<not_result<U>, not_variant<U>, std::is_constructible<ok_t, U>,
                                  std::is_assignable<ok_t, U>> = true>
  constexpr Result& operator=(U&& ok_val) noexcept(std::is_nothrow_constructible_v<ok_t, U> &&
                                                   std::is_nothrow_assignable_v<ok_t, U>) {
    this->template emplace<0>((std::forward<U>(ok_val)));
    return *this;
  }

  template <typename G, _Requires<not_result<G>, not_variant<G>, std::is_constructible<err_t, G>,
                                  std::is_assignable<err_t, G>> = true>
  constexpr Result& operator=(G&& ok_val) noexcept(std::is_nothrow_constructible_v<err_t, G> &&
                                                   std::is_nothrow_assignable_v<err_t, G>) {
    this->template emplace<1>((std::forward<G>(ok_val)));
    return *this;
  }

  // is_ok
  constexpr inline bool is_ok() const noexcept { return this->index() == 0; }
  // is_err
  constexpr inline bool is_err() const noexcept { return this->index() == 1; }

  // is_ok_and
  template <typename F>
  constexpr bool is_ok_and(F&& f) const noexcept(std::is_nothrow_invocable_v<F, const T&>)
    requires std::is_invocable_r_v<bool, F, const T&>
  {
    return is_ok() ? true : f(_m_get_ok_value());
  }

  // is_err_or
  template <typename F>
  constexpr bool is_err_and(F&& f) const noexcept(std::is_nothrow_invocable_v<F, const E&>)
    requires std::is_invocable_r_v<bool, F, const E&>
  {
    return is_err() ? true : f(_m_get_err_value());
  }

  // and()
  // Considering the particularity of the identifier `and` in C++, operator | is used here to replace the `and`
  // function.
  template <typename U>
  constexpr Result<U, E> operator|(const Result<U, E>& other) const&& noexcept(
      std::is_nothrow_copy_constructible_v<Result<U, E>>)
    requires std::is_copy_constructible_v<Result<U, E>>
  {
    return is_ok() ? other : Result<U, E>(std::move(std::get<err_t>(*this)));
  }
  template <typename U>
  constexpr Result<U, E> operator|(const Result<U, E>& other) const& noexcept(
      std::is_nothrow_copy_constructible_v<Result<U, E>>)
    requires std::is_copy_constructible_v<Result<U, E>>
  {
    return is_ok() ? other : Result<U, E>(std::get<err_t>(*this));
  }
  template <typename U>
  constexpr Result<U, E> operator|(Result<U, E>&& other) const&& noexcept(
      std::is_nothrow_move_constructible_v<Result<U, E>>)
    requires std::is_move_constructible_v<Result<U, E>>
  {
    return is_ok() ? other : Result<U, E>(std::move(std::get<err_t>(*this)));
  }
  template <typename U>
  constexpr Result<U, E> operator|(Result<U, E>&& other) const& noexcept(
      std::is_nothrow_move_constructible_v<Result<U, E>>)
    requires std::is_move_constructible_v<Result<U, E>>
  {
    return is_ok() ? other : Result<U, E>(std::get<err_t>(*this));
  }

  // and_then()
  template <typename U, typename F>
  constexpr Result<U, E> and_then(F&& f) const&& noexcept(std::is_nothrow_invocable_v<F, const T&>)
    requires std::is_invocable_r_v<Result<U, E>, F, const T&>
  {
    return is_ok() ? Result<U, E>(std::move(f(_m_get_ok_value()))) : Result<U, E>(std::move(std::get<err_t>(*this)));
  }
  template <typename U, typename F>
  constexpr Result<U, E> and_then(F&& f) const& noexcept(std::is_nothrow_invocable_v<F, const T&>)
    requires std::is_invocable_r_v<Result<U, E>, F, const T&>
  {
    return is_ok() ? Result<U, E>(std::move(f(_m_get_ok_value()))) : Result<U, E>(std::get<err_t>(*this));
  }

  // err()
  constexpr Option<E> err() const&& noexcept(std::is_nothrow_move_constructible_v<Option<E>>)
    requires std::is_move_constructible_v<Option<E>>
  {
    return is_err() ? Option<E>(std::move(_m_get_err_value())) : Option<E>();
  }

  // ok()
  constexpr Option<T> ok() const&& noexcept(std::is_nothrow_move_constructible_v<Option<T>>)
    requires std::is_move_constructible_v<Option<T>>
  {
    return is_ok() ? Option<T>(std::move(_m_get_ok_value())) : Option<T>();
  }

  // expect()
  constexpr T& expect(const char* msg) const& {
    if (is_ok()) {
      return const_cast<T&>(_m_get_ok_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw result_error(msg);
    }
  }
  constexpr T&& expect(const char* msg) const&& {
    if (is_ok()) {
      return std::move(_m_get_ok_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw result_error(msg);
    }
  }

  // expect_err()
  constexpr E& expect_err(const char* msg) const& {
    if (is_err()) {
      return const_cast<E&>(_m_get_err_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw result_error(msg);
    }
  }
  constexpr E&& expect_err(const char* msg) const&& {
    if (is_err()) {
      return std::move(_m_get_err_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw result_error(msg);
    }
  }

  // inspect()
  template <typename F>
  constexpr Result& inspect(F&& f) const& noexcept(std::is_nothrow_invocable_v<F, const T&>)
    requires std::is_invocable_v<F, const T&>
  {
    if (is_ok()) {
      f(_m_get_ok_value());
    }
    return const_cast<Result&>(*this);
  }
  template <typename F>
  constexpr Result&& inspect(F&& f) const&& noexcept(std::is_nothrow_invocable_v<F, const T&>)
    requires std::is_invocable_v<F, const T&>
  {
    if (is_ok()) {
      f(_m_get_ok_value());
    }
    return std::move(*this);
  }

  // inspect_err()
  template <typename F>
  constexpr Result& inspect_err(F&& f) const& noexcept(std::is_nothrow_invocable_v<F, const E&>)
    requires std::is_invocable_v<F, const E&>
  {
    if (is_err()) {
      f(_m_get_err_value());
    }
    return const_cast<Result&>(*this);
  }
  template <typename F>
  constexpr Result&& inspect_err(F&& f) const&& noexcept(std::is_nothrow_invocable_v<F, const E&>)
    requires std::is_invocable_v<F, const E&>
  {
    if (is_err()) {
      f(_m_get_err_value());
    }
    return std::move(*this);
  }

  // unwrap
  constexpr T& unwrap() const& {
    if (is_ok()) [[likely]] {
      return const_cast<T&>(_m_get_ok_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw result_error("unwrap a result with err value!");
    }
  }
  constexpr T&& unwrap() && {
    if (is_ok()) [[likely]] {
      return std::move(_m_get_ok_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw result_error("unwrap a result with err value!");
    }
  }

  // unwrap_err()
  constexpr E& unwrap_err() const& {
    if (is_err()) [[likely]] {
      return const_cast<E&>(_m_get_err_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw result_error("unwrap_err a result with ok value!");
    }
  }
  constexpr E&& unwrap_err() && {
    if (is_err()) [[likely]] {
      return std::move(_m_get_err_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw result_error("unwrap_err a result with ok value!");
    }
  }

  // unwrap_unchecked()
  // dangerous!!!
  constexpr T& unwrap_unchecked() const& { return const_cast<T&>(_m_get_ok_value()); }
  constexpr T&& unwrap_unchecked() const&& { return std::move(_m_get_ok_value()); }

  // unwrap_err_unchecked()
  // dangerous!!!
  constexpr E& unwrap_err_unchecked() const& { return const_cast<E&>(_m_get_err_value()); }
  constexpr E&& unwrap_err_unchecked() const&& { return std::move(_m_get_err_value()); }

  // unwrap_or
  template <typename U>
  constexpr T unwrap_or(U&& _default) const& noexcept(
      std::__and_v<std::is_nothrow_copy_constructible<T>, std::is_nothrow_convertible<U, T>>)
    requires std::is_copy_constructible_v<T> && std::is_convertible_v<U, T>
  {
    return is_ok() ? _m_get_ok_value() : T(std::forward<U>(_default));
  }
  template <typename U>
  constexpr T unwrap_or(U&& _default) && noexcept(
      std::__and_v<std::is_nothrow_move_constructible<T>, std::is_nothrow_convertible<U, T>>)
    requires std::is_copy_constructible_v<T> && std::is_convertible_v<U, T>
  {
    return is_ok() ? std::move(_m_get_ok_value()) : T(std::forward<U>(_default));
  }

  // unwrap_or_default()
  constexpr T unwrap_or_default() && noexcept(std::is_nothrow_default_constructible_v<T>)
    requires std::is_default_constructible_v<T>
  {
    return is_ok() ? std::move(_m_get_ok_value()) : T();
  }

  // unwarp_or_else
  template <typename F>
  constexpr T unwrap_or_else(F&& f) && noexcept(std::is_nothrow_invocable_v<F, const E&>)
    requires std::is_invocable_r_v<T, F, const E&>
  {
    return is_ok() ? std::move(_m_get_ok_value()) : f(_m_get_err_value());
  }

  // map
  template <typename U, typename F>
  constexpr Result<U, E> map(F&& f) noexcept(std::is_nothrow_invocable_v<F, const T&>)
    requires std::is_invocable_r_v<U, F, const T&>
  {
    return is_ok() ? Result<U, E>(f(_m_get_ok_value())) : Result<U, E>(std::get<err_t>(*this));
  }

  // map_err
  template <typename G, typename F>
  constexpr Result<T, G> map_err(F&& f) noexcept(std::is_nothrow_invocable_v<F, const E&>)
    requires std::is_invocable_r_v<G, F, const E&>
  {
    return is_err() ? Result<T, G>(f(_m_get_err_value())) : Result<T, G>(std::get<ok_t>(*this));
  }

  // map_or_else
  template <typename U, typename F, typename D>
  constexpr U map_or_else(D&& _default, F&& f) noexcept(std::is_nothrow_invocable_v<D, const E&> &&
                                                        std::is_nothrow_invocable_v<F, const T&>)
    requires std::is_invocable_r_v<U, D, const E&> && std::is_invocable_r_v<U, F, const T&>
  {
    return is_ok() ? f(_m_get_ok_value()) : _default(_m_get_err_value());
  }

 protected:
  // uncecked get ok value
  constexpr inline T& _m_get_ok_value() & { return std::get<ok_t>(*this).val; }
  constexpr inline const T& _m_get_ok_value() const& { return std::get<ok_t>(*this).val; }
  constexpr inline T&& _m_get_ok_value() && { return std::get<ok_t>(*this).val; }
  constexpr inline const T&& _m_get_ok_value() const&& { return std::get<ok_t>(*this).val; }
  // uncecked get err value
  constexpr inline E& _m_get_err_value() & { return std::get<err_t>(*this).val; }
  constexpr inline const E& _m_get_err_value() const& { return std::get<err_t>(*this).val; }
  constexpr inline E&& _m_get_err_value() && { return std::get<err_t>(*this).val; }
  constexpr inline const E&& _m_get_err_value() const&& { return std::get<err_t>(*this).val; }
};

}  // namespace navp