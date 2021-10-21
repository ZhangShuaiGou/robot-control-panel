cmake_minimum_required(VERSION 2.8.3)

project(rqt_control_panel)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp image_transport simple_grasping grasping_msgs sensor_msgs geometry_msgs rviz cv_bridge actionlib_msgs actionlib control_msgs moveit_core moveit_ros_planning_interface tf moveit_msgs pcl_conversions pcl_ros)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(rqt_control_panel_SRCS
  src/rqt_control_panel/control_panel.cpp
  src/rqt_control_panel/ratio_layouted_frame.cpp
  src/rqt_control_panel/my_rviz.cpp
)

set(rqt_control_panel_HDRS
  include/rqt_control_panel/control_panel.h
  include/rqt_control_panel/ratio_layouted_frame.h
  include/rqt_control_panel/my_rviz.h
)

set(rqt_control_panel_UIS
  src/rqt_control_panel/control_panel.ui
)

set(rqt_control_panel_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)
if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

catkin_package(
  INCLUDE_DIRS ${rqt_control_panel_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp image_transport simple_grasping grasping_msgs sensor_msgs rviz cv_bridge geometry_msgs actionlib_msgs actionlib control_msgs moveit_core moveit_ros_planning_interface tf moveit_msgs pcl_conversions pcl_ros
)
catkin_python_setup()

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_control_panel_MOCS ${rqt_control_panel_HDRS})
else()
  qt4_wrap_cpp(rqt_control_panel_MOCS ${rqt_control_panel_HDRS})
endif()
# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(rqt_control_panel_UIS_H ${rqt_control_panel_UIS})
else()
  qt4_wrap_ui(rqt_control_panel_UIS_H ${rqt_control_panel_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

include_directories(${rqt_control_panel_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_control_panel_SRCS} ${rqt_control_panel_MOCS} ${rqt_control_panel_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/rqt_control_panel
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
