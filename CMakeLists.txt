cmake_minimum_required(VERSION 3.5)
project(lcm_bridge)

#add_definitions(-std=c++11)

set(CMAKE_CXX_STANDARD 14)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

#find_package(rosidl_default_generators REQUIRED)#maybe not necessary

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(apriltag2_node REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)


###########
## Build ##
###########

#Set LCM
find_package(PkgConfig REQUIRED)
pkg_search_module(LCM REQUIRED lcm)


set(M2M_INCLUDE "$ENV{M2M_INCLUDE}")
if(M2M_INCLUDE)            
 include_directories(${M2M_INCLUDE})
else()
 message(STATUS "INCLUDE at ${M2M_INCLUDE}")
 message(FATAL_ERROR "Undefined M2M_INCLUDE env var. HINT: export M2M_INCLUDE=/path/to/mocap2mav/include")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations

find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
 include(cmake/FindEigen3.cmake)
 set(Eigen_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
${rclcpp_INCLUDE_DIRS}
${rmw_implementation_INCLUDE_DIRS}
${std_msgs_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
${nav_msgs_INCLUDE_DIRS}
${tf2_msgs_INCLUDE_DIRS}
${apriltag_msgs_INCLUDE_DIRS}
${Eigen_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(bridge_node src/bridge_node.cpp ${M2M_INCLUDE}/../src/MavState.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(bridge_node
        ${rclcpp_LIBRARIES}
        ${rmw_implementation_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${PROJECT_LINK_LIBS}
        ${LCM_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${apriltag_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${tf2_msgs_LIBRARIES}
        )

#install(TARGETS bridge_node RUNTIME DESTINATION bin)
ament_package()#maybe not
