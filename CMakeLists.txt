cmake_minimum_required(VERSION 3.14)
project(lumen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

add_executable(lumen src/main.cpp)
target_include_directories(lumen PRIVATE include)
target_link_libraries(lumen PRIVATE Threads::Threads)
target_compile_options(lumen PRIVATE -O3 -Wall -Wextra)
