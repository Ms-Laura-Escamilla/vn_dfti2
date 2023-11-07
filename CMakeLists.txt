cmake_minimum_required(VERSION 2.8.3)
project(dfti2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  vectornav
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  dftiData.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ControlSurfaceCalibration.srv

)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  vectornav
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dfti2
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dfti src/dfti.cpp)
target_link_libraries(dfti ${catkin_LIBRARIES})
add_dependencies(dfti ${catkin_EXPORTED_TARGETS})

add_executable(pixhawk_publisher src/pixhawk_publisher.cpp)
target_link_libraries(pixhawk_publisher ${catkin_LIBRARIES})
add_dependencies(pixhawk_publisher ${catkin_EXPORTED_TARGETS})

add_executable(auto_excitation src/auto_excitation.cpp)
target_link_libraries(auto_excitation ${catkin_LIBRARIES})
add_dependencies(auto_excitation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(vectornav_publisher src/vectornav_publisher.cpp)
target_link_libraries(vectornav_publisher ${catkin_LIBRARIES})
add_dependencies(vectornav_publisher ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS src/arduino_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
