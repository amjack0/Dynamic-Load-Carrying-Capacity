cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(my_nlopt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

include_directories(${PROJECT_SOURCE_DIR}/include)
#set (my_nlopt_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/include/)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  kdl_parser
)

find_package(orocos_kdl REQUIRED)
IF (orocos_kdl_FOUND)
    message(STATUS "[NLOPT] orocos_kdl found. Version: ${orocos_kdl_VERSION}")
    include_directories(BEFORE SYSTEM ${orocos_kdl_INCLUDE_DIRS})
    add_definitions(${orocos_kdl_CFLAGS})
    link_directories(${orocos_kdl_LIBRARY_DIRS})
ENDIF()

IF (NOT orocos_kdl_FOUND)
	message(STATUS "[NLOPT] orocos_kdl is not found.")
	elseif(orocos_kdl_FOUND)
	message(STATUS "[NLOPT] orocos_kdl is found.")
ENDIF() 

IF (NOT kdl_parser_FOUND)
	message(STATUS "[NLOPT] kdl_parser is not found.")
	elseif(kdl_parser_FOUND)
	message(STATUS "[NLOPT] kdl_parser is found.")
ENDIF()

find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
IF (Eigen3_FOUND)
    message(STATUS "[NLOPT] Eigen3 found. Version: ${Eigen3_VERSION}")
    include_directories(BEFORE SYSTEM ${Eigen3_INCLUDE_DIRS})
    add_definitions(${Eigen3_CFLAGS})
    link_directories(${Eigen3_LIBRARY_DIRS})
ENDIF()
link_directories(/usr/local/include)


##
find_library(MYNLOPT nlopt)
##


catkin_package(
   DEPENDS Boost
   INCLUDE_DIRS include
   LIBRARIES my_nlopt ${orocos_kdl_LIBRARIES} ${nlopt_lib}
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
   #DEPENDS system_lib
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


set(${PROJECT_NAME}_node my_nlopt_node.cpp)


add_executable(${PROJECT_NAME}_node src/my_nlopt_node.cpp 
               ${catkin_INCLUDE_DIRS})

##
find_package(PythonLibs 2.7)
include_directories(my_nlopt PRIVATE ${PYTHON_INCLUDE_DIRS})
##
               
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${orocos_kdl_LIBRARIES}
   ${kdl_parser_LIBRARIES}
   Eigen3::Eigen
   ${Boost_LIBRARIES}
   ${nlopt_lib}
   ${MYNLOPT}
   ${PYTHON_LIBRARIES})


install( DIRECTORY include/${PROJECT_NAME}/
         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )
