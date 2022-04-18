#ifndef PINGPONG__VISIBILITY_H_
#define PINGPONG__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define PINGPONG_EXPORT __attribute__ ((dllexport))
    #define PINGPONG_IMPORT __attribute__ ((dllimport))
  #else
    #define PINGPONG_EXPORT __declspec(dllexport)
    #define PINGPING_IMPORT __declspec(dllimport)
  #endif

  #ifdef PINGPONG_DLL
    #define PINGPONG_PUBLIC PINGPONG_EXPORT
  #else
    #define PINGPONG_PUBLIC PINGPONG_IMPORT
  #endif

  #define PINGPONG_PUBLIC_TYPE PINGPONG_PUBLIC

  #define PINGPONG_LOCAL

#else

  #define PINGPONG_EXPORT __attribute__ ((visibility("default")))
  #define PINGPONG_IMPORT

  #if __GNUC__ >= 4
    #define PINGPONG_PUBLIC __attribute__ ((visibility("default")))
    #define PINGPONG_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PINGPONG_PUBLIC
    #define PINGPONG_LOCAL
  #endif

  #define PINGPONG_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // PINGPONG__VISIBILITY_H_
