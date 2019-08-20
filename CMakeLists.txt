cmake_minimum_required(VERSION 2.8.3)
project(theta_star)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
)

###########
## Build ##
###########

### Theta Star Detection Library

add_library(ts_api
  src/theta_star.cpp
)

target_include_directories(ts_api
  SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(ts_api
  ${catkin_LIBRARIES}
)

add_dependencies(ts_api
  ${catkin_EXPORTED_TARGETS}
)

### ROS Interface

add_library(ros_api
  src/theta_star.cpp
  src/ros_interface_node.cpp
  src/ros_interface.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
 	src/ros_interface_node.cpp
)

target_include_directories(${PROJECT_NAME}_node
  SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME}_node
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ros_api
  ${catkin_LIBRARIES}
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # ROS Tests
  add_rostest_gtest(${PROJECT_NAME}_rostest launch/theta_star.test test/ros_interface_test.cpp)
  target_link_libraries(${PROJECT_NAME}_rostest ${catkin_LIBRARIES} ros_api)

  # Library Tests
  catkin_add_gtest(node_test test/node_test.cpp)
  catkin_add_gtest(theta_star_test test/theta_star_test.cpp)

  target_include_directories(theta_star_test
    SYSTEM PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(theta_star_test ts_api ${catkin_LIBRARIES})

  add_dependencies(theta_star_test
    ${catkin_EXPORTED_TARGETS}
  )
endif()
