cmake_minimum_required(VERSION 2.8.3)
project(rviz_mesh_plugin)

set(THIS_PACKAGE_ROS_DEPS
  roscpp
  rviz
  std_msgs
  mesh_msgs
  lvr_ros)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(catkin REQUIRED COMPONENTS 
  ${THIS_PACKAGE_ROS_DEPS} 
)

### LVR

### we are working on the lvr find package...
set(LVR_DIR "/home/robot/ros/muffin-dry/mapping/lvr")
set(LVR_INCLUDE_DIRS
  ${LVR_DIR}/ext/psimpl
  ${LVR_DIR}/ext/rply
  ${LVR_DIR}/ext/stann
  ${LVR_DIR}/ext
  ${LVR_DIR}/include/liblvr
)
set(LVR_LIBRARY_DIRS
  ${LVR_DIR}/lib
)
set(_LVR_LIBRARIES
  lvr
)
foreach(lib ${_LVR_LIBRARIES})
  find_library(${lib}_FOUND ${lib} HINTS ${LVR_LIBRARY_DIRS})
  if(${lib}_FOUND)
    list(APPEND LVR_LIBRARIES ${${lib}_FOUND})
  else()
    message( FATAL_ERROR "${lib} not found" )
  endif()
endforeach()


include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS Qt4 LVR
)


add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  include/trianglemesh_display.h
  include/face_selection_tool.h
)

set(SOURCE_FILES
  src/trianglemesh_display.cpp
  src/trianglemesh_visual.cpp
  src/face_selection_tool.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${LVR_LIBRARIES})

install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
