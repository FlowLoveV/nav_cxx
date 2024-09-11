#pragma once

#include <variant>

namespace navp {

template <typename T>
class Option;

namespace details {

struct NoneType {};

template <typename T>
concept is_tag_t = std::is_same_v<std::remove_cv_t<T>, NoneType>;
template <typename T>
concept not_tag_t = !std::is_same_v<std::remove_cv_t<T>, NoneType>;

}  // namespace details

template <details::not_tag_t T>
struct Some;

constexpr details::NoneType None{};

// The T type cannot be a tag type, which violates its original intent
template <details::not_tag_t T>
struct Some {
 private:
  template <typename _Up>
  using __not_self = std::__not_<std::is_same<Some, std::__remove_cvref_t<_Up>>>;

 public:
  typedef T value_type;

  Some() = default;
  Some(const Some&) = default;
  Some(Some&&) = default;
  Some& operator=(const Some&) = default;
  Some& operator=(Some&&) = default;

  template <details::not_tag_t U = T>
    requires __not_self<U>::value
  constexpr Some(const U& _val) noexcept(
      std::conjunction_v<std::is_nothrow_copy_constructible<T>, std::is_nothrow_constructible<T, const U&>>)
      : val(static_cast<T>(_val)) {
    static_assert(std::is_copy_constructible_v<T>);
    static_assert(std::is_constructible_v<T, const U&>);
  }

  template <details::not_tag_t U = T>
    requires __not_self<U>::value
  constexpr Some(U&& _val) noexcept(
      std::conjunction_v<std::is_nothrow_move_constructible<T>, std::is_nothrow_constructible<T, U&&>>)
      : val(static_cast<T>(std::forward<U>(_val))) {
    static_assert(std::is_move_constructible_v<T>);
    static_assert(std::is_constructible_v<T, U&&>);
  }

  template <details::not_tag_t U = T>
    requires __not_self<U>::value
  constexpr Some& operator=(const U& _val) noexcept(
      std::conjunction_v<std::is_nothrow_copy_assignable<T>, std::is_nothrow_convertible<const U&, T>>) {
    static_assert(std::is_copy_assignable_v<T>);
    static_assert(std::is_convertible_v<const U&, T>);
    if (this != &_val) {
      val = static_cast<T>(_val);
    }
    return *this;
  }

  template <details::not_tag_t U = T>
    requires __not_self<U>::value
  constexpr Some& operator=(U&& _val) noexcept(
      std::conjunction_v<std::is_nothrow_move_assignable<T>, std::is_nothrow_convertible<U&&, T>>) {
    static_assert(std::is_move_assignable_v<T>);
    static_assert(std::is_convertible_v<U&&, T>);
    if (this != &_val) {
      val = static_cast<T>(std::forward<U>(_val));
    }
    return *this;
  }

  constexpr inline operator T&() & noexcept { return val; }
  constexpr inline operator const T&() const& noexcept { return val; }
  constexpr inline operator T&&() && noexcept {
    static_assert(std::is_move_assignable_v<T>);
    return std::move(val);
  }
  constexpr inline operator const T&&() const&& noexcept {
    static_assert(std::is_move_assignable_v<T>);
    return std::move(val);
  }

  constexpr operator Option<T>() & noexcept { return Option<T>(*this); }
  constexpr operator Option<T>() && noexcept { return Option<T>(*this); }

  ~Some() = default;

  T val;
};
// deduce helper
template <typename U>
Some(U) -> Some<U>;

template <typename T>
class Option : std::variant<Some<T>, details::NoneType> {
 public:
  // operator ()
  constexpr operator bool() const noexcept { return is_some(); }

  // operator ==
  constexpr bool operator==(const Some<T>& some) const noexcept {
    if (is_some()) {
      return some.val == _m_get_some_value();
    }
    return false;
  }
  constexpr bool operator==(details::NoneType) const noexcept { return is_none(); }

  // default
  constexpr Option() noexcept : std::variant<Some<T>, details::NoneType>(details::NoneType{}) {}

  // copy/move constructor
  constexpr explicit Option(const Some<T>& some) noexcept : std::variant<Some<T>, details::NoneType>(some) {}
  constexpr explicit Option(Some<T>&& some) noexcept : std::variant<Some<T>, details::NoneType>(std::move(some)) {}
  constexpr Option(details::NoneType) noexcept : std::variant<Some<T>, details::NoneType>(details::NoneType{}) {}

  // is_some
  constexpr bool is_some() const noexcept { return this->index() == 0; }

  // is_none
  constexpr bool is_none() const noexcept { return this->index() == 1; }

  // is_some_and
  template <typename F>
    requires std::is_invocable_r_v<bool, F, const T&>
  constexpr bool is_some_and(F&& f) const& noexcept {
    if (is_some()) {
      return f(_m_get_some_value());
    }
    return false;
  }
  template <typename F>
    requires std::is_invocable_r_v<bool, F, T&&>
  constexpr bool is_some_and(F&& f) && noexcept {
    if (is_some()) {
      return f(_m_get_some_value());
    }
    return false;
  }

  // is_none_or
  template <typename F>
    requires std::is_invocable_r_v<bool, F, const T&>
  constexpr bool is_none_or(F&& f) const& noexcept {
    if (is_some()) {
      return F(_m_get_some_value());
    }
    return true;
  }
  template <typename F>
    requires std::is_invocable_r_v<bool, F, const T&&>
  constexpr bool is_none_or(F&& f) const&& noexcept {
    if (is_some()) {
      return F(_m_get_some_value());
    }
    return true;
  }

  // insert
  constexpr T& insert(const T& _val) noexcept {
    this->template emplace<0>(_val);
    return _m_get_some_value();
  }
  constexpr T& insert(T&& _val) noexcept {
    this->template emplace<0>(std::forward<T>(_val));
    return _m_get_some_value();
  }

  // get_or_insert
  constexpr T& get_or_insert(const T& _val) noexcept {
    if (is_some()) {
      return _m_get_some_value();
    } else {
      this->template emplace<0>(_val);
    }
  }
  constexpr T& get_or_insert(T&& _val) noexcept {
    if (is_some()) {
      return _m_get_some_value();
    } else {
      this->template emplace<0>(std::forward<T>(_val));
    }
  }

  // inspect
  template <typename F>
    requires std::is_invocable_r_v<void, F, const T&>
  [[nodiscard]] constexpr T& inspect(F&& f) & noexcept {
    if (is_some()) {
      f(_m_get_some_value());
    }
    return _m_get_some_value();
  }

  // replace
  

  // emplace

  // map
  // map_or
  // map_or_else

  // unwrap
  // unwrap_or
  // unwrap_or_else
  // unwrap_unchecked

  // reference, operator& -> std::reference_warpper

  // ok_or
  // ok_or_else
  // or
  // or_else

 private:
  // unchecked get value
  constexpr inline const T& _m_get_some_value() const& { return std::get<Some<T>>(*this); }
  constexpr inline T& _m_get_some_value() & { return std::get<Some<T>>(*this); }
  constexpr inline T&& _m_get_some_value() && { return std::move(std::get<Some<T>>(*this)); }
  constexpr inline const T&& _m_get_some_value() const&& { return std::move(std::get<Some<T>>(*this)); }
};

}  // namespace navp