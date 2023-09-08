cmake_minimum_required(VERSION 3.0.2)
project(test1)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  std_msgs
  message_filters
)
find_package(PCL 1.12 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test1
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#生成可执行文件
add_executable(cloud_concatenate src/cloud_concatenate.cpp)
add_executable(cloud_segment src/cloud_segment.cpp)
add_executable(hello src/hello.cpp)
add_executable(pose src/pose.cpp)
add_executable(remove_foreground src/remove_foreground.cpp)
add_executable(dynamic_filter src/dynamic_filter.cpp)
add_executable(cloud_con_dyn src/cloud_con_dyn.cpp)
# add_executable( lla2enu src/lla2enu.cpp)

#连接库
target_link_libraries(hello ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pose ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cloud_concatenate ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cloud_segment ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(remove_foreground ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(dynamic_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cloud_con_dyn ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# target_link_libraries(hello ${catkin_LIBRARIES})
# target_link_libraries(pose ${catkin_LIBRARIES})
# target_link_libraries(cloud_concatenate ${catkin_LIBRARIES})
# target_link_libraries(cloud_segment ${catkin_LIBRARIES})
# target_link_libraries(remove_foreground ${catkin_LIBRARIES})
# target_link_libraries(lla2enu ${catkin_LIBRARIES})

# target_link_libraries (hello ${PCL_LIBRARIES})
# target_link_libraries (pose ${PCL_LIBRARIES})
# target_link_libraries (cloud_concatenate ${PCL_LIBRARIES})
# target_link_libraries (cloud_segment ${PCL_LIBRARIES})
# target_link_libraries (remove_foreground ${PCL_LIBRARIES})
# target_link_libraries (lla2enu ${PCL_LIBRARIES})