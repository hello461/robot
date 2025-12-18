// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from unitree_go2_nav_interfaces:msg/NavToPose.idl
// generated code does not contain a copyright notice

#ifndef UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__STRUCT_H_
#define UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/NavToPose in the package unitree_go2_nav_interfaces.
typedef struct unitree_go2_nav_interfaces__msg__NavToPose
{
  double x;
  double y;
  double yaw;
} unitree_go2_nav_interfaces__msg__NavToPose;

// Struct for a sequence of unitree_go2_nav_interfaces__msg__NavToPose.
typedef struct unitree_go2_nav_interfaces__msg__NavToPose__Sequence
{
  unitree_go2_nav_interfaces__msg__NavToPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} unitree_go2_nav_interfaces__msg__NavToPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UNITREE_GO2_NAV_INTERFACES__MSG__DETAIL__NAV_TO_POSE__STRUCT_H_
