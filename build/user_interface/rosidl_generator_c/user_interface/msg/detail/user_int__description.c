// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from user_interface:msg/UserInt.idl
// generated code does not contain a copyright notice

#include "user_interface/msg/detail/user_int__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_user_interface
const rosidl_type_hash_t *
user_interface__msg__UserInt__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc2, 0xd9, 0xc9, 0xe2, 0x54, 0x8a, 0x88, 0xea,
      0x6f, 0x95, 0x88, 0x18, 0x1d, 0x1f, 0xda, 0x8b,
      0xa8, 0xe0, 0xaf, 0x45, 0xd7, 0x83, 0xa1, 0x82,
      0xc7, 0x54, 0x5b, 0x68, 0x76, 0x47, 0x30, 0x53,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char user_interface__msg__UserInt__TYPE_NAME[] = "user_interface/msg/UserInt";

// Define type names, field names, and default values
static char user_interface__msg__UserInt__FIELD_NAME__user_int[] = "user_int";
static char user_interface__msg__UserInt__FIELD_NAME__user_int2[] = "user_int2";
static char user_interface__msg__UserInt__FIELD_NAME__user_int3[] = "user_int3";

static rosidl_runtime_c__type_description__Field user_interface__msg__UserInt__FIELDS[] = {
  {
    {user_interface__msg__UserInt__FIELD_NAME__user_int, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {user_interface__msg__UserInt__FIELD_NAME__user_int2, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {user_interface__msg__UserInt__FIELD_NAME__user_int3, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
user_interface__msg__UserInt__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {user_interface__msg__UserInt__TYPE_NAME, 26, 26},
      {user_interface__msg__UserInt__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 user_int\n"
  "int32 user_int2\n"
  "int32 user_int3";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
user_interface__msg__UserInt__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {user_interface__msg__UserInt__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 47, 47},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
user_interface__msg__UserInt__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *user_interface__msg__UserInt__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
