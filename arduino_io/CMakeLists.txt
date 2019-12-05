cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project(arduino_io)

find_package(catkin REQUIRED COMPONENTS
  	roscpp
  	std_msgs
	std_srvs
	message_generation
  	serial
)

#add_message_files(
#    DIRECTORY
#    msg
#  FILES
#  call_grasp.msg
#grasp_status.msg
#)

# add_service_files(
#     DIRECTORY
#     srv
#   FILES
#   call_grasp.srv
# )

#generate_messages(DEPENDENCIES std_msgs std_srvs)

catkin_package(
    CATKIN_DEPENDS
    std_msgs
    message_runtime
    serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arduino_io src/arduino_io.cpp)

target_link_libraries(arduino_io
  ${catkin_LIBRARIES}
)


