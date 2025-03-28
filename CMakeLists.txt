cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing
enable_testing()

# Download and include Google Test
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)

# Add source files
set(SOURCES
    src/MathUtils.cpp
)

# Add header files
set(HEADERS
    src/MathUtils.h
)

# Create main library
add_library(physics_engine STATIC ${SOURCES} ${HEADERS})

# Add test executable
add_executable(physics_engine_tests
    tests/test_mathutils.cpp
)

# Link test executable with Google Test and our library
target_link_libraries(physics_engine_tests
    PRIVATE
    GTest::gtest_main
    GTest::gtest
    physics_engine
)

# Add test to CTest
add_test(NAME physics_engine_tests COMMAND physics_engine_tests)

# Set include directories
target_include_directories(physics_engine
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_include_directories(physics_engine_tests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
