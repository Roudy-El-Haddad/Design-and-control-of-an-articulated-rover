cmake_minimum_required(VERSION 2.8.3)

project(RobotURDF)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

catkin_package()

find_package(roslaunch)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
