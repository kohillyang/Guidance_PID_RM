cmake_minimum_required(VERSION 2.8)
project(detect_car)
find_package( OpenCV REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  roscpp
  geometry_msgs
  std_msgs
  message_generation
)
add_message_files(
  FILES
  flight_msg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dji_sdk_demo
#  CATKIN_DEPENDS dji_sdk roscpp 
#  DEPENDS dji_sdk
   CATKIN_DEPENDS message_runtime 
)
include_directories(
 	${catkin_INCLUDE_DIRS}
	./include
)
add_executable( detect_circle codes/detect_circle.cpp )
target_link_libraries( detect_circle ${catkin_LIBRARIES} ${OpenCV_LIBS} )
add_dependencies(detect_circle 
	detect_car_generate_messages_cpp)

add_executable( detect_tape codes/detect_tape.cpp )
target_link_libraries( detect_tape ${catkin_LIBRARIES} ${OpenCV_LIBS} )
add_dependencies(detect_tape 
	detect_car_generate_messages_cpp)

add_executable( detect_car codes/detect_car.cpp )
target_link_libraries( detect_car ${catkin_LIBRARIES} ${OpenCV_LIBS} )
add_dependencies(detect_car 
	detect_car_generate_messages_cpp)

add_executable( detect codes/detect.cpp )
target_link_libraries( detect ${catkin_LIBRARIES} ${OpenCV_LIBS} )
add_dependencies(detect 
	detect_car_generate_messages_cpp)




