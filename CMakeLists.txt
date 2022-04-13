cmake_minimum_required(VERSION 3.0.0)
project(serial_load_cell VERSION 0.1.0)
set (CMAKE_CXX_STANDARD 11)

include(CTest)
enable_testing()

list(APPEND CMAKE_PREFIX_PATH "/tmp/usr/local")
find_library(SERIAL_LIB serial)
message(STATUS "foo include dir: ${SERIAL_LIB}")

link_directories(/tmp/usr/local/lib/)
add_executable(main main.cpp)
target_link_libraries(main "${SERIAL_LIB}")
target_include_directories(main PRIVATE "/tmp/usr/local/include")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
