cmake_minimum_required(VERSION 3.0.2)
project(event_ros_tools)

add_compile_options(-Wall -Wextra -pedantic -Werror)

find_package(catkin REQUIRED COMPONENTS
  dvs_msgs
  prophesee_event_msgs
  event_ros_tools_msgs
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

#
# --------- flicker framer ------------
#
#add_library(flicker src/flicker.cpp)
#target_link_libraries(flicker ${OpenCV_LIBS} ${catkin_LIBRARIES})
# nodelet
#add_library(flicker_nodelet src/flicker_nodelet.cpp)
#target_link_libraries(flicker_nodelet ${OpenCV_LIBS} ${catkin_LIBRARIES})
# node
add_executable(flicker_node src/flicker_node.cpp)
target_link_libraries(flicker_node ${OPENCV_LIBS} ${catkin_LIBRARIES})


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
