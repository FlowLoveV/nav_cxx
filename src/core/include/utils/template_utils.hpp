#pragma once

#include <type_traits>

namespace navp::details {

template <typename T, template <typename...> class Template>
struct is_instance_of : std::false_type {};
template <template <typename...> class Template, typename... Args>
struct is_instance_of<Template<Args...>, Template> : std::true_type {};

}  // namespace navp::details