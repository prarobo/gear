cmake_minimum_required(VERSION 2.8.3)
project(gear_postprocess)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  gear_data_handler
)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/gear_postprocess
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)