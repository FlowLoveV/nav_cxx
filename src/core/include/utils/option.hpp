#pragma once

#include <cpptrace/cpptrace.hpp>
#include <variant>

#include "template_utils.hpp"

namespace navp {

class option_error : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

template <typename T, typename E>
class Result;

template <typename T>
class Option;

namespace details {

struct NoneType {
  explicit NoneType() = default;
};

template <typename T>
using not_tag = std::__not_<std::is_same<std::remove_cv<T>, NoneType>>;

}  // namespace details

constexpr details::NoneType None{};

template <typename _Tp, typename _Up>
using __converts_from_option =
    std::__or_<std::is_constructible<_Tp, const Option<_Up>&>, std::is_constructible<_Tp, Option<_Up>&>,
               std::is_constructible<_Tp, const Option<_Up>&&>, std::is_constructible<_Tp, Option<_Up>&&>,
               std::is_convertible<const Option<_Up>&, _Tp>, std::is_convertible<Option<_Up>&, _Tp>,
               std::is_convertible<const Option<_Up>&&, _Tp>, std::is_convertible<Option<_Up>&&, _Tp>>;

template <typename T>
class Option : private std::variant<T, details::NoneType> {
 private:
  template <typename _Up>
  using __not_self = std::__not_<std::is_same<Option, std::__remove_cvref_t<_Up>>>;
  template <typename... _Cond>
  using _Requires = std::enable_if_t<std::__and_v<_Cond...>, bool>;
  template <typename U>
  using rmcv_ref_t = std::__remove_cvref_t<U>;
  template <typename U>
  using _not = std::__not_<U>;
  template <typename... U>
  using _and = std::__and_<U...>;
  template <typename U>
  using not_option = _not<details::is_instance_of<rmcv_ref_t<U>, Option>>;
  template <typename U>
  using not_variant = _not<details::is_instance_of<rmcv_ref_t<U>, std::variant>>;

  using _Base = std::variant<T, details::NoneType>;

 public:
  // operator ()
  constexpr operator bool() const noexcept { return is_some(); }

  // operator ==
  template <typename U = T>
    requires std::is_convertible<U, T>::value
  constexpr bool operator==(const Option<U>& rhs) const noexcept(std::is_nothrow_constructible_v<U, T>) {
    if (is_some() && rhs.is_some()) {
      return rhs.unwrap_unchecked() == _m_get_some_value();
    }
    return rhs.is_none();
  }
  constexpr bool operator==(details::NoneType) const noexcept { return is_none(); }

  // operator | <=> and method in rust Option type
  template <typename U>
  constexpr Option<U> operator|(const Option<U>& rhs) const noexcept(std::is_nothrow_copy_constructible_v<Option<U>>)
    requires std::is_copy_constructible_v<Option<U>>
  {
    return is_some() ? rhs : None;
  }
  template <typename U>
  constexpr Option<U> operator|(Option<U>&& rhs) const noexcept(std::is_nothrow_move_constructible_v<Option<U>>)
    requires std::is_move_constructible_v<Option<U>>
  {
    return is_some() ? std::move(rhs) : None;
  }

  template <typename U>
  constexpr bool operator||(const Option<U>& rhs) const noexcept {
    return is_some() ? true : rhs.is_some();
  }

  template <typename U>
  constexpr bool operator&&(const Option<U>& rhs) const noexcept {
    return is_some() ? is_some() : false;
  }

  constexpr Option() noexcept : std::variant<T, details::NoneType>(details::NoneType{}) {}
  Option(const Option&) = default;
  Option(Option&&) = default;
  Option& operator=(const Option&) = default;
  Option& operator=(Option&&) = default;
  constexpr ~Option() = default;

  // construct from U value
  template <typename U = T, _Requires<__not_self<U>, details::not_tag<U>, not_option<U>, not_variant<U>,
                                      std::is_constructible<T, U>, std::is_convertible<U, T>> = true>
  constexpr Option(U&& val) noexcept(std::is_nothrow_constructible_v<T, U>)
      : std::variant<T, details::NoneType>(static_cast<T>(std::forward<U>(val))) {}

  template <typename U = T, _Requires<__not_self<U>, details::not_tag<U>, not_option<U>, not_variant<U>,
                                      std::is_constructible<T, U>, _not<std::is_convertible<U, T>>> = false>
  explicit constexpr Option(U&& val) noexcept(std::is_nothrow_constructible_v<T, U>)
      : std::variant<T, details::NoneType>(std::forward<U>(val)) {}

  // construct form Option<U>
  template <typename U, _Requires<_not<std::is_same<U, T>>, std::is_constructible<T, const U&>,
                                  std::is_convertible<const U&, T>> = true>
  constexpr Option(const Option<U>& other) noexcept(std::is_nothrow_convertible_v<T, const U&>) {
    if (other.is_none()) {
      *this = None;
    } else {
      this->template emplace<0>(other.unwrap());
      // *this = T(other.unwrap());
    }
  }

  template <typename U, _Requires<_not<std::is_same<U, T>>, std::is_constructible<T, const U&>,
                                  _not<std::is_convertible<const U&, T>>> = false>
  explicit constexpr Option(const Option<U>& other) noexcept(std::is_nothrow_convertible_v<T, const U&>) {
    if (other.is_none()) {
      *this = None;
    } else {
      this->template emplace<0>(other.unwrap());
      // *this = T(other.unwrap());
    }
  }

  template <typename U,
            _Requires<_not<std::is_same<U, T>>, std::is_constructible<T, U>, std::is_convertible<U, T>> = true>
  constexpr Option(Option<U>&& other) noexcept(std::is_nothrow_convertible_v<T, U>) {
    if (other.is_none()) {
      *this = None;
    } else {
      this->template emplace<0>(other.unwrap());
      // *this = std::move(T(other.unwrap()));
    }
  }

  template <typename U,
            _Requires<_not<std::is_same<U, T>>, std::is_constructible<T, U>, _not<std::is_convertible<U, T>>> = false>
  explicit constexpr Option(Option<U>&& other) noexcept(std::is_nothrow_convertible_v<T, U>) {
    if (other.is_none()) {
      *this = None;
    } else {
      this->template emplace<0>(other.unwrap());
      // *this = std::move(T(other.unwrap()));
    }
  }

  // construct in_place
  template <typename... Args, _Requires<std::is_constructible<T, Args...>> = false>
  explicit constexpr Option(std::in_place_t, Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>)
      : std::variant<T, details::NoneType>(std::in_place_index_t<0>{}, std::forward<Args>(args)...) {}

  template <typename U, typename... Args,
            _Requires<std::is_constructible<T, std::initializer_list<U>&, Args...>> = false>
  explicit constexpr Option(std::in_place_t, std::initializer_list<U> list, Args&&... args) noexcept(
      std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>)
      : std::variant<T, details::NoneType>(std::in_place_index_t<0>{}, list, std::forward<Args>(args)...) {}

  // from NoneType
  constexpr Option(details::NoneType) noexcept : std::variant<T, details::NoneType>(details::NoneType{}) {}

  // assign
  constexpr Option& operator=(details::NoneType) noexcept {
    *static_cast<_Base*>(this) = None;
    return *this;
  }

  template <typename U = T, _Requires<__not_self<U>, details::not_tag<U>, not_option<U>, not_variant<U>,
                                      _not<_and<std::is_scalar<T>, std::is_same<T, std::decay_t<U>>>>,
                                      std::is_constructible<T, U>, std::is_assignable<T&, U>> = true>
  constexpr Option& operator=(U&& val) noexcept(std::is_nothrow_constructible_v<T, U> &&
                                                std::is_nothrow_assignable_v<T&, U>) {
    this->template emplace<0>(std::forward<U>(val));
    return *this;
  }

  template <typename U, _Requires<_not<std::is_same<U, T>>, std::is_constructible<T, const U&>,
                                  std::is_assignable<T&, const U&>> = true>
  constexpr Option& operator=(const Option<U>& other) noexcept(std::is_nothrow_convertible_v<T, const U&> &&
                                                               std::is_nothrow_assignable_v<T&, const U&>) {
    if (other.is_none()) {
      *this = None;
    } else {
      this->template emplace<0>(other.unwrap());
    }
    return *this;
  }

  template <typename U,
            _Requires<_not<std::is_same<U, T>>, std::is_constructible<T, U>, std::is_assignable<T&, U>> = true>
  constexpr Option& operator=(Option<U>&& other) noexcept(std::is_nothrow_convertible_v<T, const U&> &&
                                                          std::is_nothrow_assignable_v<T&, U>) {
    if (other.is_none()) {
      *this = None;
    } else {
      this->template emplace<0>(std::move(other.unwrap()));
    }
    return *this;
  }

  template <typename U = T, _Requires<__not_self<U>, details::not_tag<U>,
                                      _not<details::is_instance_of<std::__remove_cvref_t<U>, Option>>,
                                      _not<details::is_instance_of<std::__remove_cvref_t<U>, std::variant>>,
                                      std::is_constructible<T, U>, std::is_convertible<U, T>> = true>
  constexpr Option& operator=(U&& val) noexcept(std::is_nothrow_constructible_v<T, U>) {}

  // is_some
  constexpr bool is_some() const noexcept { return this->index() == 0; }

  // is_none
  constexpr bool is_none() const noexcept { return this->index() == 1; }

  // is_some_and
  template <typename F>
    requires std::is_invocable_r_v<bool, F, const T&>
  constexpr bool is_some_and(F&& f) const& noexcept(std::is_nothrow_invocable_r_v<bool, F, const T&> &&
                                                    std::is_nothrow_invocable_v<F, const T&>) {
    if (is_some()) {
      return f(_m_get_some_value());
    }
    return false;
  }
  template <typename F>
    requires std::is_invocable_r_v<bool, F, T&&>
  constexpr bool is_some_and(F&& f) && noexcept(std::is_nothrow_invocable_r_v<bool, F, T&&> &&
                                                std::is_nothrow_invocable_v<F, T&&>) {
    if (is_some()) {
      return f(std::move(_m_get_some_value()));
    }
    return false;
  }

  // is_none_or
  template <typename F>
    requires std::is_invocable_r_v<bool, F, const T&>
  constexpr bool is_none_or(F&& f) const& noexcept(std::is_nothrow_invocable_v<F, const T&>) {
    if (is_some()) {
      return f(_m_get_some_value());
    }
    return true;
  }
  template <typename F>
    requires std::is_invocable_r_v<bool, F, T&&>
  constexpr bool is_none_or(F&& f) && noexcept(std::is_nothrow_invocable_v<F, T&&>) {
    if (is_some()) {
      return f(std::move(_m_get_some_value()));
    }
    return true;
  }

  // insert
  template <typename... Args>
  constexpr std::enable_if_t<std::is_constructible_v<T, Args...>, Option&> insert(Args&&... args) & noexcept(
      std::is_nothrow_constructible_v<T, Args...>) {
    this->template emplace<0>(std::forward<Args>(args)...);
    return *this;
  }
  template <typename U, typename... Args>
  constexpr std::enable_if_t<std::is_constructible_v<T, std::initializer_list<U>&, Args...>, Option&> insert(
      std::initializer_list<U> list,
      Args&&... args) & noexcept(std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>) {
    this->template emplace<0>(list, std::forward<Args>(args)...);
    return *this;
  }

  // get_or_insert
  template <typename... Args>
  constexpr std::enable_if_t<std::is_constructible_v<T, Args...>, T&> get_or_insert(Args&&... args) & noexcept(
      std::is_nothrow_constructible_v<T, Args...>) {
    if (is_none()) {
      this->template emplace<0>(std::forward<Args>(args)...);
    }
    return _m_get_some_value();
  }
  template <typename U, typename... Args>
  constexpr std::enable_if_t<std::is_constructible_v<T, std::initializer_list<U>&, Args...>, T&> get_or_insert(
      std::initializer_list<U> list,
      Args&&... args) & noexcept(std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>) {
    if (is_none()) {
      this->template emplace<0>(list, std::forward<Args>(args)...);
    }
    return _m_get_some_value();
  }

  // inspect
  template <typename F>
    requires std::is_invocable_r_v<void, F, const T&>
  [[nodiscard]] constexpr Option& inspect(F&& f) const& noexcept(std::is_nothrow_invocable_v<F, const T&>) {
    if (is_some()) {
      f(_m_get_some_value());
    }
    return const_cast<Option&>(*this);
  }
  template <typename F>
    requires std::is_invocable_r_v<void, F, const T&>
  [[nodiscard]] constexpr Option&& inspect(F&& f) const&& noexcept(std::is_nothrow_invocable_v<F, const T&>) {
    if (is_some()) {
      f(_m_get_some_value());
    }
    return std::move(*this);
  }

  // replace (usually called `emplace` in stardand libiary)
  template <typename... Args>
  constexpr std::enable_if_t<std::is_constructible_v<T, Args...>, T&> replace(Args&&... args) noexcept(
      std::is_nothrow_constructible_v<T, Args...>) {
    this->template emplace<0>(std::forward<Args>(args)...);
    return _m_get_some_value();
  }
  template <typename U, typename... Args>
  constexpr std::enable_if_t<std::is_constructible_v<T, std::initializer_list<U>&, Args...>, T&> replace(
      std::initializer_list<U> list,
      Args&&... args) noexcept(std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>) {
    this->template emplace<0>(list, std::forward<Args>(args)...);
    return _m_get_some_value();
  }

  // unwrap
  constexpr T& unwrap() const& {
    if (is_some()) {
      return const_cast<T&>(_m_get_some_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw option_error("unwrap a none option!");
    }
  }
  constexpr T&& unwrap() && {
    if (is_some()) {
      return std::move(_m_get_some_value());
    } else {
      cpptrace::generate_trace(1).print_with_snippets();
      throw option_error("unwrap a none option!");
    }
  }

  // unwrap_or
  template <typename U>
  constexpr T unwrap_or(U&& _val) const& noexcept(std::is_nothrow_copy_constructible_v<T> &&
                                                  std::is_nothrow_convertible_v<U, T>)
    requires std::is_copy_constructible_v<T> && std::is_convertible_v<U, T>
  {
    return is_some() ? _m_get_some_value() : T(std::forward<U>(_val));
  }
  template <typename U>
  constexpr T unwrap_or(U&& _val) && noexcept(std::is_nothrow_move_constructible_v<T> &&
                                              std::is_nothrow_convertible_v<U, T>)
    requires std::is_move_constructible_v<T> && std::is_convertible_v<U, T>
  {
    return is_some() ? std::move(_m_get_some_value()) : T(std::forward<T>(_val));
  }

  // unwrap_or_default
  constexpr T unwrap_or_default() && noexcept(std::is_nothrow_default_constructible_v<T> &&
                                              std::is_nothrow_move_constructible_v<T>)
    requires std::is_default_constructible_v<T> && std::is_move_constructible_v<T>
  {
    return is_some() ? std::move(_m_get_some_value()) : T();
  }

  // unwrap_or_else
  template <typename F>
    requires std::is_invocable_r_v<T, F>
  constexpr T unwrap_or_else(F&& f) && noexcept(std::is_nothrow_invocable_r_v<T, F> &&
                                                std::is_nothrow_move_constructible_v<T>)
    requires std::is_move_constructible_v<T>
  {
    return is_some() ? std::move(_m_get_some_value()) : f();
  }

  // unwrap_unchecked
  constexpr T& unwrap_unchecked() const& { return const_cast<T&>(_m_get_some_value()); }
  constexpr T&& unwrap_unchecked() && { return _m_get_some_value(); }

  // expected
  constexpr T& expect(const char* msg) const& {
    if (is_some()) {
      return const_cast<T&>(_m_get_some_value());
    }
    cpptrace::generate_trace(1).print_with_snippets();
    throw option_error(msg);
  }
  constexpr T&& expect(const char* msg) && {
    if (is_some()) {
      return std::move(_m_get_some_value());
    }
    cpptrace::generate_trace(1).print_with_snippets();
    throw option_error(msg);
  }

  // map
  template <typename F>
    requires std::is_invocable_v<F, const T&>
  constexpr auto map(F&& f) const& noexcept(std::is_nothrow_invocable_v<F, const T&>) {
    return is_some() ? f(_m_get_some_value()) : None;
  }
  template <typename F>
    requires std::is_invocable_v<F, T&&>
  constexpr auto map(F&& f) && noexcept(std::is_nothrow_invocable_v<F, T&&>) {
    return is_some() ? f(_m_get_some_value()) : None;
  }

  // map_or
  template <typename F, typename U = std::invoke_result_t<F, const T&>>
  constexpr std::invoke_result_t<F, const T&> map_or(F&& f, const U& _default) const& noexcept(
      std::is_nothrow_invocable_v<F, const T&>) {
    return is_some() ? f(_m_get_some_value()) : _default;
  }
  template <typename F, typename U = std::invoke_result_t<F, T&&>>
  constexpr std::invoke_result_t<F, const T&> map_or(F&& f,
                                                     U&& _default) && noexcept(std::is_nothrow_invocable_v<F, T&&>) {
    return is_some() ? f(std::move(_m_get_some_value())) : std::move(_default);
  }

  // map_or_else
  template <typename D, typename F, typename U = std::invoke_result_t<D>>
    requires std::is_same_v<U, std::invoke_result_t<F, const T&>>
  constexpr U map_or_else(D&& _default, F&& f) const& noexcept(std::is_nothrow_invocable_v<F, const T&> &&
                                                               std::is_nothrow_invocable_v<D>) {
    return is_some() ? f(_m_get_some_value()) : _default();
  }
  template <typename D, typename F, typename U = std::invoke_result_t<D>>
    requires std::is_same_v<U, std::invoke_result_t<F, T&&>>
  constexpr U map_or_else(D&& _default,
                          F&& f) && noexcept(std::is_nothrow_invocable_v<F, T&&> && std::is_nothrow_invocable_v<D>) {
    return is_some() ? f(std::move(_m_get_some_value())) : _default();
  }

  // as_ref
  constexpr Option<std::reference_wrapper<T>> as_ref() & noexcept {
    using rt_type = Option<std::reference_wrapper<T>>;
    return is_some() ? rt_type{std::ref(_m_get_some_value())} : Option<std::reference_wrapper<T>>{};
  }
  constexpr Option<std::reference_wrapper<T>> as_ref() const& noexcept {
    using rt_type = Option<std::reference_wrapper<T>>;
    return is_some() ? rt_type{std::cref(_m_get_some_value())} : Option<std::reference_wrapper<T>>{};
  }

  // todo list
  // ok_or
  // ok_or_else
  // or
  // or_else

 protected:
  // unchecked get value
  constexpr inline const T& _m_get_some_value() const& { return std::get<T>(*this); }
  constexpr inline T& _m_get_some_value() & { return std::get<T>(*this); }
  constexpr inline T&& _m_get_some_value() && { return std::move(std::get<T>(*this)); }
  constexpr inline const T&& _m_get_some_value() const&& { return std::move(std::get<T>(*this)); }
};

// from r value
template <typename T>
constexpr Option<T> Some(T&& _val) noexcept {
  return Option<T>(_val);
}

// from l value
template <typename T>
constexpr Option<T> Some(const T& _val) noexcept {
  return Option<T>(std::move(_val));
}

// construct in_place
template <typename T, typename... Args>
  requires std::is_constructible_v<T, Args...>
constexpr Option<T> Some(Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>) {
  return Option<T>(std::forward<Args>(args)...);
}

template <typename T, typename U, typename... Args>
  requires std::is_constructible_v<T, std::initializer_list<U>&, Args...>
constexpr Option<T> Some(std::initializer_list<U> list, Args&&... args) noexcept(
    std::is_nothrow_constructible_v<T, std::initializer_list<U>&, Args...>) {
  return Option<T>(list, std::forward<Args>(args)...);
}

}  // namespace navp