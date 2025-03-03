#ifndef HIDRAW__VISIBILITY_CONTROL_H_
#define HIDRAW__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HIDRAW_EXPORT __attribute__ ((dllexport))
    #define HIDRAW_IMPORT __attribute__ ((dllimport))
  #else
    #define HIDRAW_EXPORT __declspec(dllexport)
    #define HIDRAW_IMPORT __declspec(dllimport)
  #endif
  #ifdef HIDRAW_BUILDING_LIBRARY
    #define HIDRAW_PUBLIC HIDRAW_EXPORT
  #else
    #define HIDRAW_PUBLIC HIDRAW_IMPORT
  #endif
  #define HIDRAW_PUBLIC_TYPE HIDRAW_PUBLIC
  #define HIDRAW_LOCAL
#else
  #define HIDRAW_EXPORT __attribute__ ((visibility("default")))
  #define HIDRAW_IMPORT
  #if __GNUC__ >= 4
    #define HIDRAW_PUBLIC __attribute__ ((visibility("default")))
    #define HIDRAW_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HIDRAW_PUBLIC
    #define HIDRAW_LOCAL
  #endif
  #define HIDRAW_PUBLIC_TYPE
#endif

#endif  // HIDRAW__VISIBILITY_CONTROL_H_
