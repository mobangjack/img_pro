cmake_minimum_required(VERSION 2.8.3)
project(img_pro)

#set C++11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS 
      roscpp
      rospy
      image_transport
      cv_bridge)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

#source files
SET(SRCS 
      src/img_pro.cpp
      node/img_pro_node.cpp)

## The catkin_package macro generates cmake config files for your package
catkin_package(
      CATKIN_DEPENDS 
            roscpp 
            rospy 
            image_transport 
            cv_bridge)

## Specify additional locations of header files
include_directories(
      include
      ${OpenCV_INCLUDE_DIRS} 
      ${catkin_INCLUDE_DIRS})

link_directories(lib)

link_libraries(
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  fast)

## Declare a cpp executable for pipol_tracker_node
add_executable(img_pro ${SRCS})
#add_dependencies(img_pro ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#target_link_libraries(img_pro fast ${OpenCV_LIBS} ${catkin_LIBRARIES})

#install binaries
install(TARGETS img_pro
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#Share launch and configs
foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
