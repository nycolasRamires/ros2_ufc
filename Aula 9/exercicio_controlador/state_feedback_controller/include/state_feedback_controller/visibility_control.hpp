#ifndef STATE_FEEDBACK_CONTROLLER__VISIBILITY_CONTROL_H_
#define STATE_FEEDBACK_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define STATE_FEEDBACK_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define STATE_FEEDBACK_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define STATE_FEEDBACK_CONTROLLER_EXPORT __declspec(dllexport)
    #define STATE_FEEDBACK_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef STATE_FEEDBACK_CONTROLLER_BUILDING_LIBRARY
    #define STATE_FEEDBACK_CONTROLLER_PUBLIC STATE_FEEDBACK_CONTROLLER_EXPORT
  #else
    #define STATE_FEEDBACK_CONTROLLER_PUBLIC STATE_FEEDBACK_CONTROLLER_IMPORT
  #endif
  #define STATE_FEEDBACK_CONTROLLER_PUBLIC_TYPE STATE_FEEDBACK_CONTROLLER_PUBLIC
  #define STATE_FEEDBACK_CONTROLLER_LOCAL
#else
  #define STATE_FEEDBACK_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define STATE_FEEDBACK_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define STATE_FEEDBACK_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define STATE_FEEDBACK_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define STATE_FEEDBACK_CONTROLLER_PUBLIC
    #define STATE_FEEDBACK_CONTROLLER_LOCAL
  #endif
  #define STATE_FEEDBACK_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // STATE_FEEDBACK_CONTROLLER__VISIBILITY_CONTROL_H_
