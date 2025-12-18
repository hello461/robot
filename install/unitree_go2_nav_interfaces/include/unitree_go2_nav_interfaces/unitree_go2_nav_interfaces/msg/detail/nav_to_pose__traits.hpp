// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unitree_go2_nav_interfaces:msg/NavToPose.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__TRAITS_HPP_
#define UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "unitree_go2_nav_interfaces/msg/detail/nav_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace unitree_go2_nav_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const NavToPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavToPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavToPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace unitree_go2_nav_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use unitree_go2_nav_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const unitree_go2_nav_interfaces::msg::NavToPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  unitree_go2_nav_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use unitree_go2_nav_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const unitree_go2_nav_interfaces::msg::NavToPose & msg)
{
  return unitree_go2_nav_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<unitree_go2_nav_interfaces::msg::NavToPose>()
{
  return "unitree_go2_nav_interfaces::msg::NavToPose";
}

template<>
inline const char * name<unitree_go2_nav_interfaces::msg::NavToPose>()
{
  return "unitree_go2_nav_interfaces/msg/NavToPose";
}

template<>
struct has_fixed_size<unitree_go2_nav_interfaces::msg::NavToPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<unitree_go2_nav_interfaces::msg::NavToPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<unitree_go2_nav_interfaces::msg::NavToPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__TRAITS_HPP_
