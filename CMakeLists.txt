cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
project(tr_hub_parser)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)
find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(cfg/tr_hub_parser.cfg)

add_message_files(
    FILES
    RangeArray.msg
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime dynamic_reconfigure
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(tr_hub_parser src/tr_hub_parser.cpp src/serial_port.cpp)
target_link_libraries(tr_hub_parser ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(tr_hub_parser tr_hub_parser_generate_messages_cpp)
add_dependencies(tr_hub_parser ${${PROJECT_NAME}_EXPORTED_TARGETS})


 if(CATKIN_ENABLE_TESTING)
    add_library(${PROJECT_NAME}_LIB src/tr_hub_parser.cpp src/serial_port.cpp)
    target_link_libraries(${PROJECT_NAME}_LIB ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    add_dependencies(tr_hub_parser_LIB tr_hub_parser_generate_messages_cpp)

    catkin_add_gtest(tr_hub_parser_test test/test_tr_hub_parser.cpp)
    add_dependencies(tr_hub_parser_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(tr_hub_parser_test ${Boost_LIBRARIES} ${GTEST_LIBRARIES} ${PROJECT_NAME}_LIB)
endif()
