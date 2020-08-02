cmake_minimum_required(VERSION 2.8.3)
project(path_planner)
add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES path_planner
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(velocity_plan src/velocity_plan.cpp)
target_link_libraries(velocity_plan ${catkin_LIBRARIES})

add_executable(localization src/localization.cpp)
target_link_libraries(localization ${catkin_LIBRARIES})
