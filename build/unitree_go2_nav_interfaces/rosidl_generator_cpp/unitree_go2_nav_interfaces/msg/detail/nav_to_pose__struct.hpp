// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unitree_go2_nav_interfaces:msg/NavToPose.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__STRUCT_HPP_
#define UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__unitree_go2_nav_interfaces__msg__NavToPose __attribute__((deprecated))
#else
# define DEPRECATED__unitree_go2_nav_interfaces__msg__NavToPose __declspec(deprecated)
#endif

namespace unitree_go2_nav_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavToPose_
{
  using Type = NavToPose_<ContainerAllocator>;

  explicit NavToPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->yaw = 0.0;
    }
  }

  explicit NavToPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unitree_go2_nav_interfaces__msg__NavToPose
    std::shared_ptr<unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unitree_go2_nav_interfaces__msg__NavToPose
    std::shared_ptr<unitree_go2_nav_interfaces::msg::NavToPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavToPose_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavToPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavToPose_

// alias to use template instance with default allocator
using NavToPose =
  unitree_go2_nav_interfaces::msg::NavToPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unitree_go2_nav_interfaces

#endif  // UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__STRUCT_HPP_
