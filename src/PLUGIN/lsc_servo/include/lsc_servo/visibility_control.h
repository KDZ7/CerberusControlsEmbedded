#ifndef LSC_SERVO__VISIBILITY_CONTROL_H_
#define LSC_SERVO__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LSC_SERVO_EXPORT __attribute__ ((dllexport))
    #define LSC_SERVO_IMPORT __attribute__ ((dllimport))
  #else
    #define LSC_SERVO_EXPORT __declspec(dllexport)
    #define LSC_SERVO_IMPORT __declspec(dllimport)
  #endif
  #ifdef LSC_SERVO_BUILDING_LIBRARY
    #define LSC_SERVO_PUBLIC LSC_SERVO_EXPORT
  #else
    #define LSC_SERVO_PUBLIC LSC_SERVO_IMPORT
  #endif
  #define LSC_SERVO_PUBLIC_TYPE LSC_SERVO_PUBLIC
  #define LSC_SERVO_LOCAL
#else
  #define LSC_SERVO_EXPORT __attribute__ ((visibility("default")))
  #define LSC_SERVO_IMPORT
  #if __GNUC__ >= 4
    #define LSC_SERVO_PUBLIC __attribute__ ((visibility("default")))
    #define LSC_SERVO_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LSC_SERVO_PUBLIC
    #define LSC_SERVO_LOCAL
  #endif
  #define LSC_SERVO_PUBLIC_TYPE
#endif

#endif  // LSC_SERVO__VISIBILITY_CONTROL_H_
