cmake_minimum_required(VERSION 2.8.3)

project(haptic_control)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs turtlesim message_filters Eigen3)

catkin_package()

find_package(roslaunch)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(include 
	${catkin_INCLUDE_DIRS}
)
	
add_executable(omega_vel_cmd src/omega_vel_cmd.cpp)
target_link_libraries(omega_vel_cmd ${catkin_LIBRARIES}
	dhd
	usb-1.0 -lrt
	pthread)
	
add_executable(sigma_vel_cmd src/sigma_vel_cmd.cpp)
target_link_libraries(sigma_vel_cmd ${catkin_LIBRARIES}
	dhd
	usb-1.0 -lrt
	pthread)
	
foreach(dir config launch meshes urdf rviz)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
