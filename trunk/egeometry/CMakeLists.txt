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
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(${PROJECT_NAME} src/coordinates.cpp src/cascaded_coordinates.cpp)

rosbuild_add_gtest(test/test_matrix test/test_matrix.cpp)
rosbuild_add_gtest(test/test_vector test/test_vector.cpp)

rosbuild_add_gtest(test/test_coordinates test/test_coordinates.cpp)
#add_dependencies(test/test_coordinates ${PROJECT_NAME})
target_link_libraries(test/test_coordinates ${PROJECT_NAME})

rosbuild_add_gtest(test/test_msg test/test_msg.cpp)
target_link_libraries(test/test_msg ${PROJECT_NAME})

rosbuild_add_gtest(test/test_cascoords test/test_cascoords.cpp)
target_link_libraries(test/test_cascoords ${PROJECT_NAME})

rosbuild_add_executable(test/viewer test/viewer.cpp)
target_link_libraries(test/viewer glut ${PROJECT_NAME})

rosbuild_add_executable(test/test_publish test/test_publish.cpp)
add_dependencies(test/test_publish ${PROJECT_NAME})
target_link_libraries(test/test_publish ${PROJECT_NAME})
