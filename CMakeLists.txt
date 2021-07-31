cmake_minimum_required(VERSION 3.0.2)
project(event_ros_tools)

find_package(catkin REQUIRED COMPONENTS
  dvs_msgs
  prophesee_event_msgs
  roscpp
  nodelet
  image_transport
  cv_bridge
  )

find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


#
# --------- event slicer ------------
#
# nodelet
add_library(slicer_nodelet src/slicer_nodelet.cpp)
target_link_libraries(slicer_nodelet ${OpenCV_LIBS} ${catkin_LIBRARIES})
# node
add_executable(slicer_node src/slicer_node.cpp)
target_link_libraries(slicer_node ${OpenCV_LIBS} ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS slicer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS slicer_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
 
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  )


#############
## Testing ##
#############

# To be done...
