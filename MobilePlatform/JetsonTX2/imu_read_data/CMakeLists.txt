cmake_minimum_required(VERSION 2.8.3)
project(imu_read_data)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  serial
  roscpp
  std_msgs
)

  add_message_files(
    FILES
    msgImu.msg
  )
  generate_messages(
    DEPENDENCIES
    std_msgs
  )

catkin_package(
   LIBRARIES 
    imu_read_data

   CATKIN_DEPENDS
    serial 
    roscpp 
    std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_read src/imu_read.cpp)
target_link_libraries(imu_read ${catkin_LIBRARIES})
add_dependencies(imu_read imu_read_data_generate_messages_cpp)
