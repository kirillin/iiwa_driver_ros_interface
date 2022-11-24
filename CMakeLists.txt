cmake_minimum_required(VERSION 3.0.2)
project(iiwa_driver_ros_interface)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation 
  geometry_msgs
)

add_message_files(FILES 
	JointPositions.msg
	JointTorques.msg
	JointVelocities.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(drake REQUIRED)
find_package(lcm REQUIRED)

find_package(Boost COMPONENTS system thread REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_control
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/iiwa_driver_ros_interface
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(iiwa_driver_ros_interface_node src/iiwa_driver_ros_interface.cpp)
target_link_libraries(iiwa_driver_ros_interface_node ${catkin_LIBRARIES} ${LCM_NAMESPACE}lcm ${gflags_LIBRARIES} drake::drake-lcmtypes-cpp)


# add_executable(${PROJECT_NAME}_node src/loop.cpp)
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   ${fri_client_library} ${LCM_NAMESPACE}lcm ${gflags_LIBRARIES} drake::drake-lcmtypes-cpp Eigen3::Eigen
# )