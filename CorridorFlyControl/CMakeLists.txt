cmake_minimum_required(VERSION 2.4.6)
project(CorridorFlyControl)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	cmake_modules	
	ardrone_autonomy 	 
	)

include_directories(${catkin_INCLUDE_DIRS}
		)

SET(SOURCES ${PROJECT_SOURCE_DIR}/src/main.cpp
            )

#SET(HEADERS ${PROJECT_SOURCE_DIR}/include/viewpoint_estimation_lib.h)

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp 
  LIBRARIES
  roscpp
  cmake_modules
)

add_executable(${PROJECT_NAME} ${SOURCES} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES}
)


