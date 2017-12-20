cmake_minimum_required (VERSION 2.8)
project(csc232)
set (CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR} CACHE PATH "default install prefix" FORCE)
find_program(GENGETOPT gengetopt)


execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/ros_publisher.ggo --file-name ros_publisher_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)

execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/ros_subscriber.ggo --file-name ros_subscriber_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)

#execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/Localizer.ggo --file-name Localizer_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)

execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/Localization.ggo --file-name Localization_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)

set(ROS_PATH_PREFIX /opt/ros/indigo)
find_path(ROS_INCLUDE_DIR NAMES ros/ros.h HINTS CMAKE_INSTALL_PREFIX/include)

include_directories(${ROS_INCLUDE_DIR} ${PROJECT_BINARY_DIR})
file(GLOB ROS_LIBRARIES "${ROS_PATH_PREFIX}/lib/*.so*")

add_executable(ros_publisher ros_publisher.cpp ${CMAKE_CURRENT_BINARY_DIR}/ros_publisher_cmdline.c)
add_executable(ros_subscriber ros_subscriber.cpp ${CMAKE_CURRENT_BINARY_DIR}/ros_subscriber_cmdline.c)
#add_executable(Localizer Localizer.cpp ${CMAKE_CURRENT_BINARY_DIR}/Localizer_cmdline.c)
add_executable(Localization Localization.cpp Localizer.cpp point.cpp ${MOC_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/Localization_cmdline.c)
add_executable(pfc pfc.cpp point.cpp ${MOC_SRCS})
add_executable(executive executive.cpp point.cpp ${MOC_SRCS})
add_executable(planner planner.cpp point.cpp ${MOC_SRCS})
#add_executable(pfc pfc.cpp ${MOC_SRCS})



target_link_libraries(ros_publisher ${ROS_LIBRARIES})
target_link_libraries(ros_subscriber ${ROS_LIBRARIES})
#target_link_libraries(Localizer ${ROS_LIBRARIES})
target_link_libraries(Localization ${ROS_LIBRARIES})
#target_link_libraries(Point ${ROS_LIBRARIES})
target_link_libraries(pfc ${ROS_LIBRARIES})
target_link_libraries(executive ${ROS_LIBRARIES})
target_link_libraries(planner ${ROS_LIBRARIES})



install(TARGETS ros_publisher DESTINATION bin)
install(TARGETS ros_subscriber DESTINATION bin)
#install(TARGETS Localizer DESTINATION bin)
install(TARGETS Localization DESTINATION bin)
#install(TARGETS Point DESTINATION bin)
install(TARGETS pfc DESTINATION bin)
install(TARGETS executive DESTINATION bin)
install(TARGETS planner DESTINATION bin)

#find Eigen libraries
SET(ENV{EIGEN3_INCLUDE_DIR} "/usr/include/eigen3")
SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
#include_directories( "$ENV{EIGEN3_INCLUDE_DIR}" )

# find the QT libraries
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
find_package( OpenGL REQUIRED )
find_package( GLUT REQUIRED )
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} )
# generate the MOC_SRCS from the QT header
qt_wrap_cpp(gui MOC_SRCS gui.h)
# generate the header and source files from the gengetopt file
execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/gui_process.ggo --file-name gui_process_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)
# generate the executable and link the libraries
add_executable(gui-process gui_process.cpp gui.cpp point.cpp ${MOC_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/gui_process_cmdline.c) 
target_link_libraries(gui-process ${ROS_LIBRARIES} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} )
install( TARGETS gui-process DESTINATION bin )

