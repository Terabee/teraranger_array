cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
project(teraranger_array)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  dynamic_reconfigure
  serial
)
find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(
cfg/TerarangerHubOne.cfg
cfg/TerarangerHubMultiflex.cfg
cfg/TerarangerHubEvo.cfg
)

add_message_files(
    FILES
    RangeArray.msg
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime dynamic_reconfigure serial geometry_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(teraranger_one src/teraranger_one.cpp src/helper_lib.cpp)
target_link_libraries(teraranger_one ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(teraranger_one ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(teraranger_one ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(teraranger_one ${PROJECT_NAME}_gencfg)

add_executable(teraranger_multiflex src/teraranger_multiflex.cpp src/helper_lib.cpp)
target_link_libraries(teraranger_multiflex ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(teraranger_multiflex ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(teraranger_multiflex ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(teraranger_multiflex ${PROJECT_NAME}_gencfg)

add_executable(teraranger_evo src/teraranger_evo.cpp src/helper_lib.cpp)
target_link_libraries(teraranger_evo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(teraranger_evo ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(teraranger_evo ${catkin_EXPORTED_TARGETS})
add_dependencies(teraranger_evo ${PROJECT_NAME}_gencfg)

install( TARGETS
  teraranger_one
  teraranger_multiflex
  teraranger_evo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

 if(CATKIN_ENABLE_TESTING)
    add_library(${PROJECT_NAME}_LIB src/teraranger_one.cpp src/helper_lib.cpp)
    target_link_libraries(${PROJECT_NAME}_LIB ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    add_dependencies(${PROJECT_NAME}_LIB ${PROJECT_NAME}_generate_messages_cpp)
    add_dependencies(${PROJECT_NAME}_LIB ${PROJECT_NAME}_gencfg)

    catkin_add_gtest(teraranger_one_test test/test_teraranger_one.cpp src/helper_lib.cpp)
    add_dependencies(teraranger_one_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(teraranger_one_test ${Boost_LIBRARIES} ${GTEST_LIBRARIES} ${PROJECT_NAME}_LIB)
endif()
