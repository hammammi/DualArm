when make cmakelist.txt that has msg file

order is matter.

# declare a cpp executable
add_executable(mobile_controller src/mobile_controller.cpp)
# add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
add_dependencies(mobile_controller vehicle_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
# specify libraries to link a library or executable target against
target_link_libraries(mobile_controller ${catkin_LIBRARIES})

