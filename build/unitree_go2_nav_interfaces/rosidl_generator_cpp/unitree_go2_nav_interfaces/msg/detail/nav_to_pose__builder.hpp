// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unitree_go2_nav_interfaces:msg/NavToPose.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__BUILDER_HPP_
#define UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "unitree_go2_nav_interfaces/msg/detail/nav_to_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace unitree_go2_nav_interfaces
{

namespace msg
{

namespace builder
{

class Init_NavToPose_yaw
{
public:
  explicit Init_NavToPose_yaw(::unitree_go2_nav_interfaces::msg::NavToPose & msg)
  : msg_(msg)
  {}
  ::unitree_go2_nav_interfaces::msg::NavToPose yaw(::unitree_go2_nav_interfaces::msg::NavToPose::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unitree_go2_nav_interfaces::msg::NavToPose msg_;
};

class Init_NavToPose_y
{
public:
  explicit Init_NavToPose_y(::unitree_go2_nav_interfaces::msg::NavToPose & msg)
  : msg_(msg)
  {}
  Init_NavToPose_yaw y(::unitree_go2_nav_interfaces::msg::NavToPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_NavToPose_yaw(msg_);
  }

private:
  ::unitree_go2_nav_interfaces::msg::NavToPose msg_;
};

class Init_NavToPose_x
{
public:
  Init_NavToPose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavToPose_y x(::unitree_go2_nav_interfaces::msg::NavToPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_NavToPose_y(msg_);
  }

private:
  ::unitree_go2_nav_interfaces::msg::NavToPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unitree_go2_nav_interfaces::msg::NavToPose>()
{
  return unitree_go2_nav_interfaces::msg::builder::Init_NavToPose_x();
}

}  // namespace unitree_go2_nav_interfaces

#endif  // UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__BUILDER_HPP_
