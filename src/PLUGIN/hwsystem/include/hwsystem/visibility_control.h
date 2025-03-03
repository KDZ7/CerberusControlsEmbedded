#ifndef HWSYSTEM__VISIBILITY_CONTROL_H_
#define HWSYSTEM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HWSYSTEM_EXPORT __attribute__ ((dllexport))
    #define HWSYSTEM_IMPORT __attribute__ ((dllimport))
  #else
    #define HWSYSTEM_EXPORT __declspec(dllexport)
    #define HWSYSTEM_IMPORT __declspec(dllimport)
  #endif
  #ifdef HWSYSTEM_BUILDING_LIBRARY
    #define HWSYSTEM_PUBLIC HWSYSTEM_EXPORT
  #else
    #define HWSYSTEM_PUBLIC HWSYSTEM_IMPORT
  #endif
  #define HWSYSTEM_PUBLIC_TYPE HWSYSTEM_PUBLIC
  #define HWSYSTEM_LOCAL
#else
  #define HWSYSTEM_EXPORT __attribute__ ((visibility("default")))
  #define HWSYSTEM_IMPORT
  #if __GNUC__ >= 4
    #define HWSYSTEM_PUBLIC __attribute__ ((visibility("default")))
    #define HWSYSTEM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HWSYSTEM_PUBLIC
    #define HWSYSTEM_LOCAL
  #endif
  #define HWSYSTEM_PUBLIC_TYPE
#endif

#endif  // HWSYSTEM__VISIBILITY_CONTROL_H_
