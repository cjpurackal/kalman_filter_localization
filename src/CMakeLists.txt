cmake_minimum_required(VERSION 3.1.0)
project(kf)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/")


#find_package(cv_bridge_1_13 REQUIRED)
#set(OpenCV_DIR "/home/developer/co-fusion/deps/opencv-3.1.0/build")
#set(cv_bridge_DIR "/home/developer/packages/vision_opencv/cv_bridge/install/share/cv_bridge/cmake")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg cv_bridge image_transport tf2 tf2_geometry_msgs tf2_sensor_msgs tf2_ros tf2_eigen) 

include_directories(${catkin_INCLUDE_DIRS}) 
catkin_package()

find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Warnings / Errors
add_compile_options(-std=c++14 -Wall -Wno-unused-function -Wno-unused-variable
  -Wno-unused-but-set-variable -Wno-unused-but-set-variable -Wno-write-strings
  -Wno-deprecated -Wno-deprecated-declarations -Wno-unknown-pragmas)

# Don't follow symlinks when FILE GLOB_RECURSE (and don't warn)
cmake_policy(SET CMP0009 NEW)


# Eigen3
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(${Eigen3_INCLUDE_DIRS})



# project-wide paths
include_directories(${PROJECT_SOURCE_DIR})


file(GLOB src *.cpp)


add_executable(${PROJECT_NAME}
               ${src}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    ${Eigen_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pthread
  )

