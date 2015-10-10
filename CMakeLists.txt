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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${QT_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS Qt4
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
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
