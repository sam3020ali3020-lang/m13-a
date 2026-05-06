# ==============================================================================
# Native Race-Detection Tests — Host build for SharedSensorData stress testing
# ==============================================================================
# يُبنى على Linux host (ليس Android NDK) لاستخدام TSan + ASan.
# يختبر نمط الـ synchronization لـ `SharedSensorData` — نفس نمط الإنتاج.
#
# Usage:
#   cmake -S . -B build_tsan -DSANITIZER=thread
#   cmake -S . -B build_asan -DSANITIZER=address
#   cmake -S . -B build_release -DSANITIZER=none
# ==============================================================================

cmake_minimum_required(VERSION 3.16)
project(native_race_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -------------------------------------------------------------------
# خيار sanitizer
# -------------------------------------------------------------------
set(SANITIZER "none" CACHE STRING "Sanitizer to enable: none, thread, address, undefined")
set_property(CACHE SANITIZER PROPERTY STRINGS none thread address undefined)

message(STATUS "Sanitizer: ${SANITIZER}")

if(SANITIZER STREQUAL "thread")
    add_compile_options(-fsanitize=thread -g -O1 -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
elseif(SANITIZER STREQUAL "address")
    add_compile_options(-fsanitize=address -fsanitize=undefined -g -O1 -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address -fsanitize=undefined)
elseif(SANITIZER STREQUAL "undefined")
    add_compile_options(-fsanitize=undefined -g -O1 -fno-omit-frame-pointer)
    add_link_options(-fsanitize=undefined)
elseif(SANITIZER STREQUAL "none")
    add_compile_options(-O2 -g)
else()
    message(FATAL_ERROR "Unknown SANITIZER: ${SANITIZER}")
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# Negative-control flag: build a deliberately racy test to verify TSan works
option(PROVOKE_RACE "Add a deliberately racy test to verify sanitizer works" OFF)
if(PROVOKE_RACE)
    add_compile_definitions(PROVOKE_RACE=1)
    message(STATUS "PROVOKE_RACE enabled — sanitizer should abort")
endif()

# -------------------------------------------------------------------
# GoogleTest
# -------------------------------------------------------------------
find_package(GTest REQUIRED)

# -------------------------------------------------------------------
# paths
# -------------------------------------------------------------------
# نصل لـ shared_sensor_data.h الحقيقي بدون نسخ
get_filename_component(CPP_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../app/src/main/cpp" ABSOLUTE)

# shim لـ <drivers/drv_hrt.h>
set(SHIMS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/shims")

message(STATUS "Using shared_sensor_data.h from: ${CPP_SRC_DIR}")
message(STATUS "Using PX4 shims from: ${SHIMS_DIR}")

# -------------------------------------------------------------------
# هدف الاختبار
# -------------------------------------------------------------------
add_executable(stress_sensor_race
    stress_sensor_race.cpp
)

target_include_directories(stress_sensor_race PRIVATE
    ${CPP_SRC_DIR}       # shared_sensor_data.h
    ${SHIMS_DIR}         # drivers/drv_hrt.h
)

target_link_libraries(stress_sensor_race PRIVATE
    GTest::GTest
    GTest::Main
    pthread
)

# -------------------------------------------------------------------
# CTest integration
# -------------------------------------------------------------------
enable_testing()
add_test(NAME stress_sensor_race COMMAND stress_sensor_race)

# TSan/ASan options عبر environment variables
if(SANITIZER STREQUAL "thread")
    set_tests_properties(stress_sensor_race PROPERTIES
        ENVIRONMENT "TSAN_OPTIONS=halt_on_error=1:second_deadlock_stack=1:history_size=7"
    )
elseif(SANITIZER STREQUAL "address")
    set_tests_properties(stress_sensor_race PROPERTIES
        ENVIRONMENT "ASAN_OPTIONS=halt_on_error=1:detect_leaks=1:abort_on_error=0;UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1"
    )
endif()
