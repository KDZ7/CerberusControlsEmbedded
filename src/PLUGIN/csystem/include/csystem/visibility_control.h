#ifndef CSYSTEM__VISIBILITY_CONTROL_H_
#define CSYSTEM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CSYSTEM_EXPORT __attribute__ ((dllexport))
    #define CSYSTEM_IMPORT __attribute__ ((dllimport))
  #else
    #define CSYSTEM_EXPORT __declspec(dllexport)
    #define CSYSTEM_IMPORT __declspec(dllimport)
  #endif
  #ifdef CSYSTEM_BUILDING_LIBRARY
    #define CSYSTEM_PUBLIC CSYSTEM_EXPORT
  #else
    #define CSYSTEM_PUBLIC CSYSTEM_IMPORT
  #endif
  #define CSYSTEM_PUBLIC_TYPE CSYSTEM_PUBLIC
  #define CSYSTEM_LOCAL
#else
  #define CSYSTEM_EXPORT __attribute__ ((visibility("default")))
  #define CSYSTEM_IMPORT
  #if __GNUC__ >= 4
    #define CSYSTEM_PUBLIC __attribute__ ((visibility("default")))
    #define CSYSTEM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CSYSTEM_PUBLIC
    #define CSYSTEM_LOCAL
  #endif
  #define CSYSTEM_PUBLIC_TYPE
#endif

#endif  // CSYSTEM__VISIBILITY_CONTROL_H_
