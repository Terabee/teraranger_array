cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
project(tr_hub_parser)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  #tr_hub_parser
)
# since we need boost search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(cfg/tr_hub_parser.cfg)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    RangeArray.msg
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime dynamic_reconfigure 
  DEPENDS boost
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# make the executables
add_executable(tr_hub_parser src/tr_hub_parser.cpp src/serial_port.cpp)
target_link_libraries(tr_hub_parser ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(tr_hub_parser ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Make the library for the test
add_library(${PROJECT_NAME}_LIB src/tr_hub_parser.cpp src/serial_port.cpp)
target_link_libraries(${PROJECT_NAME}_LIB ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Adding tests
catkin_add_gtest(tr_hub_parser_test test/test_tr_hub_parser.cpp)
add_dependencies(tr_hub_parser_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tr_hub_parser_test ${Boost_LIBRARIES} ${GTEST_LIBRARIES} ${PROJECT_NAME}_LIB)
