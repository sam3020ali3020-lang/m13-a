#
# Toolchain file for cross-compiling acados to Android ARM64 (aarch64)
# Uses the Android NDK's built-in toolchain file
#
# Usage:
#   cmake -DCMAKE_TOOLCHAIN_FILE=<path>/Toolchain-android-arm64.cmake \
#         -DANDROID_NDK=<ndk_path> ..
#

# Allow overriding NDK path via environment or command line
if(NOT DEFINED ANDROID_NDK)
    if(DEFINED ENV{ANDROID_NDK})
        set(ANDROID_NDK $ENV{ANDROID_NDK})
    elseif(DEFINED ENV{ANDROID_NDK_HOME})
        set(ANDROID_NDK $ENV{ANDROID_NDK_HOME})
    else()
        # Default path on this system
        set(ANDROID_NDK "/home/yoga/android-ndk/android-ndk-r27c")
    endif()
endif()

# Validate NDK path
if(NOT EXISTS "${ANDROID_NDK}/build/cmake/android.toolchain.cmake")
    message(FATAL_ERROR
        "Android NDK toolchain not found at: ${ANDROID_NDK}/build/cmake/android.toolchain.cmake\n"
        "Set ANDROID_NDK to the path of your Android NDK installation.")
endif()

# Android target configuration
set(ANDROID_ABI "arm64-v8a" CACHE STRING "Android ABI")
set(ANDROID_PLATFORM "android-26" CACHE STRING "Android API level")
set(ANDROID_STL "c++_static" CACHE STRING "Android STL")
set(ANDROID_ARM_NEON ON CACHE BOOL "Enable NEON")

# Include the NDK's toolchain file (does the heavy lifting)
include("${ANDROID_NDK}/build/cmake/android.toolchain.cmake")
