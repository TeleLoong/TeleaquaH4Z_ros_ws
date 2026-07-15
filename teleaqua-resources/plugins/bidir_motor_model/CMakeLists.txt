cmake_minimum_required(VERSION 3.10)
project(bidir_motor_model)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Protobuf REQUIRED) # for gz::msgs

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(gz-sim-bidir-motor-model-system SHARED BiDirectionalMotorModel.cpp)
target_link_libraries(gz-sim-bidir-motor-model-system
  PUBLIC
    gz-plugin2::register
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-math7::gz-math7
    gz-common5::gz-common5
    gz-msgs10::gz-msgs10
)

target_include_directories(gz-sim-bidir-motor-model-system PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS gz-sim-bidir-motor-model-system
  LIBRARY DESTINATION lib)


