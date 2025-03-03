#ifndef HWSYSTEM_AG__VISIBILITY_CONTROL_H_
#define HWSYSTEM_AG__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HWSYSTEM_AG_EXPORT __attribute__ ((dllexport))
    #define HWSYSTEM_AG_IMPORT __attribute__ ((dllimport))
  #else
    #define HWSYSTEM_AG_EXPORT __declspec(dllexport)
    #define HWSYSTEM_AG_IMPORT __declspec(dllimport)
  #endif
  #ifdef HWSYSTEM_AG_BUILDING_LIBRARY
    #define HWSYSTEM_AG_PUBLIC HWSYSTEM_AG_EXPORT
  #else
    #define HWSYSTEM_AG_PUBLIC HWSYSTEM_AG_IMPORT
  #endif
  #define HWSYSTEM_AG_PUBLIC_TYPE HWSYSTEM_AG_PUBLIC
  #define HWSYSTEM_AG_LOCAL
#else
  #define HWSYSTEM_AG_EXPORT __attribute__ ((visibility("default")))
  #define HWSYSTEM_AG_IMPORT
  #if __GNUC__ >= 4
    #define HWSYSTEM_AG_PUBLIC __attribute__ ((visibility("default")))
    #define HWSYSTEM_AG_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HWSYSTEM_AG_PUBLIC
    #define HWSYSTEM_AG_LOCAL
  #endif
  #define HWSYSTEM_AG_PUBLIC_TYPE
#endif

#endif  // HWSYSTEM_AG__VISIBILITY_CONTROL_H_
