cmake_minimum_required(VERSION 2.8.3)
project(LC)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if(WIN32 AND NOT CYGWIN)
    set(HOME $ENV{PROFILE})
else()
    set(HOME $ENV{HOME})
endif()
 
find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 
 sensor_msgs
)
 
 
catkin_package(CATKIN_DEPENDS
roscpp
sensor_msgs
)
 
include_directories(
include
${catkin_INCLUDE_DIRS}
)
 
 
### find files
 
file(GLOB_RECURSE HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h*)
file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c*)
 
add_executable(LaserChooser
${HEADER_FILES}
${SOURCE_FILES}
)
 
target_link_libraries(LaserChooser
${catkin_LIBRARIES}
)

add_dependencies(LaserChooser ${catkin_EXPORTED_TARGETS})
 
