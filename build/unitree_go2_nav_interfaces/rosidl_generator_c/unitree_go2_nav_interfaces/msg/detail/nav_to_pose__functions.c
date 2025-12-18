// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from unitree_go2_nav_interfaces:msg/NavToPose.idl
// generated code does not contain a copyright notice
#include "unitree_go2_nav_interfaces/msg/detail/nav_to_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
unitree_go2_nav_interfaces__msg__NavToPose__init(unitree_go2_nav_interfaces__msg__NavToPose * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // yaw
  return true;
}

void
unitree_go2_nav_interfaces__msg__NavToPose__fini(unitree_go2_nav_interfaces__msg__NavToPose * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // yaw
}

bool
unitree_go2_nav_interfaces__msg__NavToPose__are_equal(const unitree_go2_nav_interfaces__msg__NavToPose * lhs, const unitree_go2_nav_interfaces__msg__NavToPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
unitree_go2_nav_interfaces__msg__NavToPose__copy(
  const unitree_go2_nav_interfaces__msg__NavToPose * input,
  unitree_go2_nav_interfaces__msg__NavToPose * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // yaw
  output->yaw = input->yaw;
  return true;
}

unitree_go2_nav_interfaces__msg__NavToPose *
unitree_go2_nav_interfaces__msg__NavToPose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go2_nav_interfaces__msg__NavToPose * msg = (unitree_go2_nav_interfaces__msg__NavToPose *)allocator.allocate(sizeof(unitree_go2_nav_interfaces__msg__NavToPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(unitree_go2_nav_interfaces__msg__NavToPose));
  bool success = unitree_go2_nav_interfaces__msg__NavToPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
unitree_go2_nav_interfaces__msg__NavToPose__destroy(unitree_go2_nav_interfaces__msg__NavToPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    unitree_go2_nav_interfaces__msg__NavToPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
unitree_go2_nav_interfaces__msg__NavToPose__Sequence__init(unitree_go2_nav_interfaces__msg__NavToPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go2_nav_interfaces__msg__NavToPose * data = NULL;

  if (size) {
    data = (unitree_go2_nav_interfaces__msg__NavToPose *)allocator.zero_allocate(size, sizeof(unitree_go2_nav_interfaces__msg__NavToPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = unitree_go2_nav_interfaces__msg__NavToPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        unitree_go2_nav_interfaces__msg__NavToPose__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
unitree_go2_nav_interfaces__msg__NavToPose__Sequence__fini(unitree_go2_nav_interfaces__msg__NavToPose__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      unitree_go2_nav_interfaces__msg__NavToPose__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

unitree_go2_nav_interfaces__msg__NavToPose__Sequence *
unitree_go2_nav_interfaces__msg__NavToPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  unitree_go2_nav_interfaces__msg__NavToPose__Sequence * array = (unitree_go2_nav_interfaces__msg__NavToPose__Sequence *)allocator.allocate(sizeof(unitree_go2_nav_interfaces__msg__NavToPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = unitree_go2_nav_interfaces__msg__NavToPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
unitree_go2_nav_interfaces__msg__NavToPose__Sequence__destroy(unitree_go2_nav_interfaces__msg__NavToPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    unitree_go2_nav_interfaces__msg__NavToPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
unitree_go2_nav_interfaces__msg__NavToPose__Sequence__are_equal(const unitree_go2_nav_interfaces__msg__NavToPose__Sequence * lhs, const unitree_go2_nav_interfaces__msg__NavToPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!unitree_go2_nav_interfaces__msg__NavToPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
unitree_go2_nav_interfaces__msg__NavToPose__Sequence__copy(
  const unitree_go2_nav_interfaces__msg__NavToPose__Sequence * input,
  unitree_go2_nav_interfaces__msg__NavToPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(unitree_go2_nav_interfaces__msg__NavToPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    unitree_go2_nav_interfaces__msg__NavToPose * data =
      (unitree_go2_nav_interfaces__msg__NavToPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!unitree_go2_nav_interfaces__msg__NavToPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          unitree_go2_nav_interfaces__msg__NavToPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!unitree_go2_nav_interfaces__msg__NavToPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
