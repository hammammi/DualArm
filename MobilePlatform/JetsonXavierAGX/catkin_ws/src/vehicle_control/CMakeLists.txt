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
    jointstatesMsg.msg
    posMsgs.msg
    gnd.msg
    positionMsg.msg
  )


## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
     std_msgs
  )


catkin_package(
   LIBRARIES vehicle_control
   CATKIN_DEPENDS
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


add_executable(mobile_controller src/mobile_controller.cpp)
add_dependencies(mobile_controller vehicle_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(mobile_controller ${catkin_LIBRARIES})


add_executable(encoder_actual src/encoder_actual.cpp)
add_dependencies(encoder_actual vehicle_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(encoder_actual ${catkin_LIBRARIES})


add_executable(experiment_rectangle src/experiment_rectangle.cpp)
add_dependencies(experiment_rectangle vehicle_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(experiment_rectangle ${catkin_LIBRARIES})


add_executable(experiment_diamond src/experiment_diamond.cpp)
add_dependencies(experiment_diamond vehicle_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(experiment_diamond ${catkin_LIBRARIES})


add_executable(vel_cmd src/vel_cmd.cpp)
add_dependencies(vel_cmd vehicle_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(vel_cmd ${catkin_LIBRARIES})


add_executable(vel_cmd_circle_fix src/vel_cmd_circle_fix.cpp)
add_dependencies(vel_cmd_circle_fix vehicle_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(vel_cmd_circle_fix ${catkin_LIBRARIES})

add_executable(experiment_route1 src/experiment_route1.cpp)
target_link_libraries(experiment_route1 ${catkin_LIBRARIES})

add_executable(experiment_route2 src/experiment_route2.cpp)
target_link_libraries(experiment_route2 ${catkin_LIBRARIES})
