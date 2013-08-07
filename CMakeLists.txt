cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
find_package(message_generation) #for custom message
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
INCLUDE (FindCURL) 
#ros_add_boost_directories()

#rosbuild_add_executable(Kalman_filter src/kalman.cpp)
rosbuild_add_executable(Drone_test src/Drone_test.cpp)
rosbuild_add_executable(Roomba_test src/Roomba_test.cpp)
rosbuild_add_executable(Rotate_tag src/rotate_tag.cpp)
rosbuild_add_executable(Point2Point src/Point2Point.cpp)
#rosbuild_add_executable(Listen src/csv_listener.cpp)
#rosbuild_add_executable(Quuppa_decode src/Quuppa_decode.cpp)
rosbuild_add_executable(AZ_SIM src/AZ_sim.cpp)

rosbuild_add_executable(Quuppa_AZ src/Quuppa_AZ.cpp)
#rosbuild_link_boost(Quuppa_AZ algo

#rosbuild_add_executable(Json src/json_decode.cpp)
#target_link_libraries(Json ${CURL_LIBRARIES}) 
#target_link_libraries(Json jansson) 

