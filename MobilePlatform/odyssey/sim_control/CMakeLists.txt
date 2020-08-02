cmake_minimum_required(VERSION 2.8.3)
project(sim_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  tf
  nav_msgs
  sensor_msgs
)

add_message_files(
  FILES
  desiredMsg.msg
  motorDynamics.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sim_control
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_executable(tracking_controller_prev src/tracking_controller_prev.cpp)
target_link_libraries(tracking_controller_prev ${catkin_LIBRARIES})
add_dependencies(tracking_controller_prev sim_control_generate_messages_cpp)

add_executable(tracking_pd_controller src/tracking_pd_controller.cpp)
target_link_libraries(tracking_pd_controller ${catkin_LIBRARIES})
add_dependencies(tracking_pd_controller sim_control_generate_messages_cpp)

add_executable(motor_vel_profile src/motor_vel_profile.cpp)
target_link_libraries(motor_vel_profile ${catkin_LIBRARIES})
add_dependencies(motor_vel_profile sim_control_generate_messages_cpp)

