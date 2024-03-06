#ifndef PI3HAT_MOTEUS_INT_MSGS__VISIBILITY_CONTROL_H_
#define PI3HAT_MOTEUS_INT_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PI3HAT_MOTEUS_INT_MSGS_EXPORT __attribute__ ((dllexport))
    #define PI3HAT_MOTEUS_INT_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define PI3HAT_MOTEUS_INT_MSGS_EXPORT __declspec(dllexport)
    #define PI3HAT_MOTEUS_INT_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PI3HAT_MOTEUS_INT_MSGS_BUILDING_LIBRARY
    #define PI3HAT_MOTEUS_INT_MSGS_PUBLIC PI3HAT_MOTEUS_INT_MSGS_EXPORT
  #else
    #define PI3HAT_MOTEUS_INT_MSGS_PUBLIC PI3HAT_MOTEUS_INT_MSGS_IMPORT
  #endif
  #define PI3HAT_MOTEUS_INT_MSGS_PUBLIC_TYPE PI3HAT_MOTEUS_INT_MSGS_PUBLIC
  #define PI3HAT_MOTEUS_INT_MSGS_LOCAL
#else
  #define PI3HAT_MOTEUS_INT_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define PI3HAT_MOTEUS_INT_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define PI3HAT_MOTEUS_INT_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define PI3HAT_MOTEUS_INT_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PI3HAT_MOTEUS_INT_MSGS_PUBLIC
    #define PI3HAT_MOTEUS_INT_MSGS_LOCAL
  #endif
  #define PI3HAT_MOTEUS_INT_MSGS_PUBLIC_TYPE
#endif
#endif  // PI3HAT_MOTEUS_INT_MSGS__VISIBILITY_CONTROL_H_
// Generated 16-Feb-2024 11:14:16
 