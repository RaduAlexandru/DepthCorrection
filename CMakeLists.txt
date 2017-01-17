cmake_minimum_required(VERSION 2.8)
project(depthCalib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "-O3")
set (CMAKE_CXX_STANDARD 11)



find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	image_transport
	ecl_threads
	cv_bridge
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}" )
    set (CMAKE_CXX_RelFLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}" )
endif()

FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(QGLViewer)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package( OpenCV REQUIRED )
find_package(PCL 1.8 REQUIRED) 
include_directories( ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})


set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )

catkin_package()




link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} )
SET(my_FORMS src/fancyWindow.ui)
QT4_WRAP_UI(my_FORMS_HEADERS ${my_FORMS})
SET(my_HEADERS src/fancyWindow.h src/fancyViewer.h src/mySubscriber.h src/Plane.h src/FancyQueue.h src/shared/CalibrationMatrix.h src/PCL_normEst.h)
QT4_WRAP_CPP(my_HEADERS_MOC ${my_HEADERS})
INCLUDE(${QT_USE_FILE})

find_package(Boost REQUIRED COMPONENTS
		thread
		filesystem
                signals
)



add_executable(offlineCorrection src/offline/offlineCorrection.cpp src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})
#add_executable(crossValidation src/crossValidation/crossValidation.cpp src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})
add_executable(rosToPGM src/batch/rosToPGM.cpp src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})
add_executable(batchCalib src/batch/batchCalib.cpp src/batch/batchCalib.h src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})
add_executable(downsampler src/downsampler/downsample.cpp src/shared/CalibrationMatrix.h src/shared/CalibrationMatrix.cpp ${PCL_INCLUDE_DIRS})
add_executable(xtionCalibratedDriver   src/driver/main.cpp
                                                src/driver/driver.h
                                                src/driver/driver.cpp
                                                src/shared/CalibrationMatrix.h
src/shared/CalibrationMatrix.cpp )

add_executable(standardViewer  src/main.cpp
                                        src/fancyWindow.cpp
                                        src/fancyViewer.cpp
                                        src/mySubscriber.cpp
                                        src/FancyQueue.cpp
                                        src/Plane.cpp
                                        src/shared/CalibrationMatrix.cpp
${my_FORMS_HEADERS} ${my_HEADERS_MOC} )

TARGET_LINK_LIBRARIES(standardViewer  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  ${QGLVIEWER_LIBRARY_RELEASE} ${QT_LIBRARIES} ${PCL_INCLUDE_DIRS} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${Boost_LIBRARIES}  )


target_link_libraries (offlineCorrection ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries (rosToPGM ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (batchCalib ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries (downsampler ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries (xtionCalibratedDriver ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${catkin_LIBRARIES})

