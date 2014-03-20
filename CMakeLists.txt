cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin/")
rosbuild_init()
rosbuild_add_boost_directories()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_BUILD_TYPE Release)
#SET(laser_SOURCES src/main.cpp src/laserWindow.cpp)
#SET(laser_HEADERS src/laserWindow.h)
#SET(laser_FORMS src/laserWindow.ui)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}" )
    set (CMAKE_CXX_RelFLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
endif()


#FIND_LIBRARY( OPENNI2_LIBRARY
#             NAMES OpenNI2
#             HINTS ${PC_OPENNI2_LIBDIR} ${PC_OPENNI2_LIBRARY_DIRS} /usr/lib
#             PATHS "$ENV{PROGRAMFILES}/OpenNI2/Lib${OPENNI2_SUFFIX}" "$ENV{PROGRAMW6432}/OpenNI2/Lib${OPENNI2_SUFFIX}" "$ENV{PROGRAMW6432}/OpenNI2"
#             PATH_SUFFIXES lib lib64
#)
#FIND_PATH( OPENNI2_INCLUDE_DIR OpenNI.h
#          HINTS ${PC_OPENNI2_INCLUDEDIR} ${PC_OPENNI2_INCLUDE_DIRS}
#                  /usr/include/openni2 /usr/include/ni2
#                  PATHS "$ENV{PROGRAMFILES}/OpenNI2/include" "$ENV{PROGRAMW6432}/OpenNI2/include"
#          PATH_SUFFIXES openni2 ni2)





#find_package(Eigen3 REQUIRED)
#include_directories(EIGEN3_INCLUDE_DIR)
#find_package(PCL 1.7 REQUIRED)
#include_directories(BEFORE ${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(QGLViewer)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} )
INCLUDE_DIRECTORIES("/home/malcom/src/ros/OpenNI2/ThirdParty/GL/")
SET(my_FORMS src/fancyWindow.ui)
QT4_WRAP_UI(my_FORMS_HEADERS ${my_FORMS})
SET(my_HEADERS src/fancyWindow.h src/fancyViewer.h src/mySubscriber.h src/Plane.h src/FancyQueue.h src/shared/CalibrationMatrix.h src/PCL_normEst.h)
QT4_WRAP_CPP(my_HEADERS_MOC ${my_HEADERS})
INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})

#rosbuild_add_executable(xtionToPCDZF src/xtionToPCDZF/xtionToPCDZF.cpp ${PCL_INCLUDE_DIRS} ${OPENNI2_INCLUDE_DIR} ${OPENNI2_LIBRARY}
#                    src/xtionToPCDZF/Viewer.h src/xtionToPCDZF/Viewer.cpp
#                    src/xtionToPCDZF/OniSampleUtilities.h
#
#)
#link_directories( ${OPENNI2_LIBRARY}  )
#INCLUDE_DIRECTORIES( ${OPENNI2_INCLUDE_DIR}  )
#target_link_libraries( xtionToPCDZF ${OPENNI2_LIBRARY} )
#target_link_libraries(xtionToPCDZF ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
#SET_TARGET_PROPERTIES(xtionToPCDZF PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
#rosbuild_link_boost(xtionToPCDZF thread signals system)



rosbuild_add_executable(offlineCorrection src/offline/offlineCorrection.cpp src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})

rosbuild_add_executable(crossValidation src/crossValidation/crossValidation.cpp src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})
#SET_TARGET_PROPERTIES(offlineCorrection PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})


rosbuild_add_executable(rosToPGM src/batch/rosToPGM.cpp src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})
#SET_TARGET_PROPERTIES(rosToPGM PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})

rosbuild_add_executable(batchCalib src/batch/batchCalib.cpp src/batch/batchCalib.h src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})
#SET_TARGET_PROPERTIES(batchCalib PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})

rosbuild_add_executable(downsampler src/downsampler/downsample.cpp src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})
#SET_TARGET_PROPERTIES(downsampler PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})

rosbuild_add_executable(xtionCalibratedDriver   src/driver/main.cpp
                                                src/driver/driver.h
                                                src/driver/driver.cpp
                                                src/shared/CalibrationMatrix.h
                                                src/shared/CalibrationMatrix.cpp )

rosbuild_link_boost(xtionCalibratedDriver thread signals system)
#SET_TARGET_PROPERTIES(downsampler PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})

rosbuild_add_executable(standardViewer  src/main.cpp
                                        src/fancyWindow.cpp
                                        src/fancyViewer.cpp
                                        src/mySubscriber.cpp
                                        src/FancyQueue.cpp
                                        src/Plane.cpp
                                        src/shared/CalibrationMatrix.cpp
                                        ${my_FORMS_HEADERS} ${my_HEADERS_MOC} ${PCL_INCLUDE_DIRS})

#SET_TARGET_PROPERTIES(standardViewer PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
TARGET_LINK_LIBRARIES(standardViewer ${QGLVIEWER_LIBRARY_RELEASE} ${QT_LIBRARIES} ${PCL_INCLUDE_DIRS} )
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
