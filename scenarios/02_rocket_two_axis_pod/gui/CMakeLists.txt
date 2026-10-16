cmake_minimum_required(VERSION 3.17)
project(TargetControl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(gz-gui8 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)

add_library(TargetControl SHARED
  TargetControl.cc
  TargetControl.hh
  TargetControl.qrc)
target_link_libraries(TargetControl PRIVATE
  gz-gui8::gz-gui8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  Qt5::Core)

add_library(TargetTrajectory SHARED TargetTrajectory.cc)
target_link_libraries(TargetTrajectory PRIVATE
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
set_target_properties(TargetTrajectory PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/target-system-plugins")

include(CTest)
if(BUILD_TESTING)
  find_package(Qt5 REQUIRED COMPONENTS Test Quick)
  add_executable(TargetControl_test TargetControl_test.cc)
  target_link_libraries(TargetControl_test PRIVATE
    TargetControl
    gz-gui8::gz-gui8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    Qt5::Test
    Qt5::Quick)
  add_test(NAME TargetControl COMMAND TargetControl_test)
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/test-home")
  set_tests_properties(TargetControl PROPERTIES ENVIRONMENT
    "HOME=${CMAKE_BINARY_DIR}/test-home;QT_QPA_PLATFORM=offscreen;QT_QUICK_BACKEND=software;GZ_GUI_PLUGIN_PATH=$<TARGET_FILE_DIR:TargetControl>;GZ_PARTITION=target_control_test")
endif()
