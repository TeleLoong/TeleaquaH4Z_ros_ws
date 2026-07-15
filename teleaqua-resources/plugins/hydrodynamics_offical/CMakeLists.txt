cmake_minimum_required(VERSION 3.10)
project(hydrodynamics)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED COMPONENTS eigen3)
find_package(gz-common5 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Protobuf REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(hydrodynamics SHARED hydrodynamics.cpp)
target_link_libraries(hydrodynamics
  PUBLIC
    gz-plugin2::register
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-math7::gz-math7
    gz-common5::gz-common5
    gz-msgs10::gz-msgs10
)

target_include_directories(hydrodynamics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS hydrodynamics
  LIBRARY DESTINATION lib)