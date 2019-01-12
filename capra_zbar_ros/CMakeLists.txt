cmake_minimum_required(VERSION 2.8.3)
project(capra_zbar_ros)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  nodelet
  roscpp
  capra_zbar_msgs
  image_transport
)

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  nodelet
  roscpp
  zbar
  capra_zbar_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/reader_nodelet.cpp)
add_dependencies(${PROJECT_NAME} capra_zbar_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}
  zbar
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)