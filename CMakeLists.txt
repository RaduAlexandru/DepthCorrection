cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin/")
rosbuild_init()
rosbuild_add_boost_directories()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_BUILD_TYPE Debug)
#SET(laser_SOURCES src/main.cpp src/laserWindow.cpp)
#SET(laser_HEADERS src/laserWindow.h)
#SET(laser_FORMS src/laserWindow.ui)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}" )
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
endif()

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(QGLViewer)


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
SET(my_FORMS src/fancyWindow.ui)
QT4_WRAP_UI(my_FORMS_HEADERS ${my_FORMS})
SET(my_HEADERS src/fancyWindow.h src/fancyViewer.h src/mySubscriber.h src/Plane.h src/FancyQueue.h src/shared/CalibrationMatrix.h)
QT4_WRAP_CPP(my_HEADERS_MOC ${my_HEADERS})
INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})

rosbuild_add_executable(standardViewer    src/main.cpp
                                        src/fancyWindow.cpp
                                        src/fancyViewer.cpp
                                        src/mySubscriber.cpp
                                        src/FancyQueue.cpp
                                        src/Plane.cpp
                                        ${my_FORMS_HEADERS} ${my_HEADERS_MOC})


rosbuild_add_executable(offlineCalibration src/offline/offlineCalibration.cpp src/shared/CalibrationMatrix.h )

TARGET_LINK_LIBRARIES(standardViewer ${QGLVIEWER_LIBRARY_RELEASE} ${QT_LIBRARIES} )
rosbuild_link_boost(standardViewer thread signals system)


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
