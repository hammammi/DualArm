cmake_minimum_required(VERSION 2.8.3)
project(vehicle_control)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  tf
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  nodelet
)

  add_message_files(
    FILES
    commendMsg.msg
    motorsMsg.msg
    motorMsg.msg
    jointstatesMsg.msg
    posMsgs.msg
    tfMsg.msg
    gnd.msg
    noise.msg
    wMsg.msg
    positionMsg.msg
  )


## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
     std_msgs
  )


catkin_package(
   LIBRARIES vehicle_control
   CATKIN_DEPENDS message_runtime
    geometry_msgs
    tf
    nav_msgs
    roscpp
    std_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_publish src/cmd_publish.cpp)
target_link_libraries(cmd_publish ${catkin_LIBRARIES})
add_dependencies(cmd_publish vehicle_control_generate_messages_cpp)

add_executable(vehicle_controller src/vehicle_controller.cpp)
target_link_libraries(vehicle_controller ${catkin_LIBRARIES})
add_dependencies(vehicle_controller vehicle_control_generate_messages_cpp)


add_executable(vehicle_controller_2 src/vehicle_controller_2.cpp)
target_link_libraries(vehicle_controller_2 ${catkin_LIBRARIES})

add_executable(kalman_node src/kalman_node.cpp)
target_link_libraries(kalman_node ${catkin_LIBRARIES})
add_dependencies(kalman_node vehicle_control_generate_messages_cpp)


add_executable(encoder_node2 src/encoder_node2.cpp)
target_link_libraries(encoder_node2 ${catkin_LIBRARIES})
add_dependencies(encoder_node2 vehicle_control_generate_messages_cpp)


add_executable(encoder_node3 src/encoder_node3.cpp)
target_link_libraries(encoder_node3 ${catkin_LIBRARIES})
add_dependencies(encoder_node3 vehicle_control_generate_messages_cpp)


add_executable(encoder_node4 src/encoder_node4.cpp)
target_link_libraries(encoder_node4 ${catkin_LIBRARIES})
add_dependencies(encoder_node4 vehicle_control_generate_messages_cpp)



add_executable(kalman_comb src/kalman_comb.cpp)
target_link_libraries(kalman_comb ${catkin_LIBRARIES})
add_dependencies(kalman_comb vehicle_control_generate_messages_cpp)


add_executable(motor_dynamics src/motor_dynamics.cpp)
target_link_libraries(motor_dynamics ${catkin_LIBRARIES})
add_dependencies(motor_dynamics vehicle_control_generate_messages_cpp)

add_executable(motor_dynamics_v2 src/motor_dynamics_v2.cpp)
target_link_libraries(motor_dynamics_v2 ${catkin_LIBRARIES})
add_dependencies(motor_dynamics_v2 vehicle_control_generate_messages_cpp)



add_executable(cmd_publish_2 src/cmd_publish_2.cpp)
target_link_libraries(cmd_publish_2 ${catkin_LIBRARIES})
add_dependencies(cmd_publish_2 vehicle_control_generate_messages_cpp)


add_executable(cmd_publish_3 src/cmd_publish_3.cpp)
target_link_libraries(cmd_publish_3 ${catkin_LIBRARIES})
add_dependencies(cmd_publish_3 vehicle_control_generate_messages_cpp)


add_executable(exp1_1 src/exp1_1.cpp)
target_link_libraries(exp1_1 ${catkin_LIBRARIES})
add_dependencies(exp1_1 vehicle_control_generate_messages_cpp)

add_executable(exp1_2 src/exp1_2.cpp)
target_link_libraries(exp1_2 ${catkin_LIBRARIES})
add_dependencies(exp1_2 vehicle_control_generate_messages_cpp)


add_executable(exp2 src/exp2.cpp)
target_link_libraries(exp2 ${catkin_LIBRARIES})
add_dependencies(exp2 vehicle_control_generate_messages_cpp)


add_executable(exp3_1 src/exp3_1.cpp)
target_link_libraries(exp3_1 ${catkin_LIBRARIES})
add_dependencies(exp3_1 vehicle_control_generate_messages_cpp)

add_executable(exp3_2 src/exp3_2.cpp)
target_link_libraries(exp3_2 ${catkin_LIBRARIES})
add_dependencies(exp3_2 vehicle_control_generate_messages_cpp)



add_executable(vel_cmd src/vel_cmd.cpp)
target_link_libraries(vel_cmd ${catkin_LIBRARIES})
add_dependencies(vel_cmd vehicle_control_generate_messages_cpp)


add_executable(vel_cmd_circle_fix src/vel_cmd_circle_fix.cpp)
target_link_libraries(vel_cmd_circle_fix ${catkin_LIBRARIES})
add_dependencies(vel_cmd_circle_fix vehicle_control_generate_messages_cpp)

add_executable(encoder_actual src/encoder_actual.cpp)
target_link_libraries(encoder_actual ${catkin_LIBRARIES})
add_dependencies(encoder_actual encoder_actual_generate_messages_cpp)

add_executable(experiment_route1 src/experiment_route1.cpp)
target_link_libraries(experiment_route1 ${catkin_LIBRARIES})
add_dependencies(experiment_route1 experiment_route1_generate_messages_cpp)

add_executable(experiment_route2 src/experiment_route2.cpp)
target_link_libraries(experiment_route2 ${catkin_LIBRARIES})
add_dependencies(experiment_route2 experiment_route2_generate_messages_cpp)



