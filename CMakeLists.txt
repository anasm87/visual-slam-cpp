cmake_minimum_required(VERSION 3.20)
project(visual_slam_cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Integrate with vcpkg toolchain
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
endif()
 
# Strict compiler warnings
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Disable OpenCV/Eigen tests to prevent test floods (if fetched or built from source)
set(BUILD_TESTING OFF CACHE BOOL "Disable testing for dependencies" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Disable Eigen testing" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Disable OpenCV tests" FORCE)
set(BUILD_PERF_TESTS OFF CACHE BOOL "Disable OpenCV perf tests" FORCE)

# Find vcpkg dependencies
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

# Re-enable testing for our own tests
set(BUILD_TESTING ON CACHE BOOL "Enable testing for our project" FORCE)

# Include directories
include_directories(include)

# Create visual odometry library
add_library(visual_odometry src/visual_odometry.cpp)
target_include_directories(visual_odometry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(visual_odometry PUBLIC Eigen3::Eigen ${OpenCV_LIBS})

# Google Test via FetchContent
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(test_vo tests/test_vo.cpp)
target_link_libraries(test_vo PRIVATE visual_odometry GTest::gtest_main)
add_test(NAME test_vo COMMAND test_vo)
