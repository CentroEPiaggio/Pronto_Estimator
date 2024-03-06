#ifndef PRONTO_MSGS__VISIBILITY_CONTROL_H_
#define PRONTO_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PRONTO_MSGS_EXPORT __attribute__ ((dllexport))
    #define PRONTO_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define PRONTO_MSGS_EXPORT __declspec(dllexport)
    #define PRONTO_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PRONTO_MSGS_BUILDING_LIBRARY
    #define PRONTO_MSGS_PUBLIC PRONTO_MSGS_EXPORT
  #else
    #define PRONTO_MSGS_PUBLIC PRONTO_MSGS_IMPORT
  #endif
  #define PRONTO_MSGS_PUBLIC_TYPE PRONTO_MSGS_PUBLIC
  #define PRONTO_MSGS_LOCAL
#else
  #define PRONTO_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define PRONTO_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define PRONTO_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define PRONTO_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PRONTO_MSGS_PUBLIC
    #define PRONTO_MSGS_LOCAL
  #endif
  #define PRONTO_MSGS_PUBLIC_TYPE
#endif
#endif  // PRONTO_MSGS__VISIBILITY_CONTROL_H_
// Generated 16-Feb-2024 11:14:18
 