/*
    Gathered directly from the tutorial in the ROS documentation, which mentions:
    "In order to make the package compile and work on Windows, we need to add in some “visibility
   control”."
*/
#ifndef TELEOPDATA___VISIBILITY_CONTROL_H_
#define TELEOPDATA___VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

    // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
    //     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
        #define TELEOPDATA__EXPORT __attribute__((dllexport))
        #define TELEOPDATA__IMPORT __attribute__((dllimport))
    #else
        #define TELEOPDATA__EXPORT __declspec(dllexport)
        #define TELEOPDATA__IMPORT __declspec(dllimport)
    #endif
    #ifdef TELEOPDATA__BUILDING_DLL
        #define TELEOPDATA__PUBLIC TELEOPDATA__EXPORT
    #else
        #define TELEOPDATA__PUBLIC TELEOPDATA__IMPORT
    #endif
    #define TELEOPDATA__PUBLIC_TYPE TELEOPDATA__PUBLIC
    #define TELEOPDATA__LOCAL
#else
    #define TELEOPDATA__EXPORT __attribute__((visibility("default")))
    #define TELEOPDATA__IMPORT
    #if __GNUC__ >= 4
        #define TELEOPDATA__PUBLIC __attribute__((visibility("default")))
        #define TELEOPDATA__LOCAL __attribute__((visibility("hidden")))
    #else
        #define TELEOPDATA__PUBLIC
        #define TELEOPDATA__LOCAL
    #endif
    #define TELEOPDATA__PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif // TELEOPDATA___VISIBILITY_CONTROL_H_
