# Find or fetch GoogleTest
find_package(GTest QUIET)

if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0)
  FetchContent_MakeAvailable(googletest)
endif()

# Test files
file(GLOB_RECURSE TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

if(NOT TEST_SOURCES)
  message(WARNING "No test source files found in ${CMAKE_CURRENT_SOURCE_DIR}")
  return()
endif()

# Test executable
add_executable(unit_tests ${TEST_SOURCES})

# Link against your core library and gtest
target_link_libraries(unit_tests PRIVATE ${PROJECT_NAME}_lib GTest::gtest_main)

# Include directories
target_include_directories(unit_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)

# In tests/CMakeLists.txt, replace gtest_discover_tests with:
include(GoogleTest)
gtest_discover_tests(unit_tests DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 10)
