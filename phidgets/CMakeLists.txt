cmake_minimum_required(VERSION 2.8.3)
project(phidgets)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  geometry_msgs
  move_base_msgs
  nav_msgs
  std_msgs
  roscpp
  rviz
  sensor_msgs
  sound_play
  std_msgs
  tf
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
textlcd_params.msg
stepper_params.msg
spatial_params.msg
servo_params.msg
rfid_params.msg
pid_params.msg
phsensor_params.msg
pantilt_params.msg
motor_params.msg
manager_params.msg
led_params.msg
joystick_params.msg
ir_params.msg
interface_kit_params.msg
encoder_params.msg
accelerometer_params.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
interface_kit.srv
lidar_control.srv
pid.srv
servo_reference.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   move_base_msgs   nav_msgs   sensor_msgs   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phidgets
#  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs nav_msgs OpenCV roscpp rviz sensor_msgs sound_play std_msgs tf
  DEPENDS system_lib message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(phidgets
#   src/${PROJECT_NAME}/phidgets.cpp
# )

## Declare a cpp executable
# add_executable(phidgets_node src/phidgets_node.cpp)
add_executable(simple_servo src/simple_servo.cpp)
add_executable(advanced_servo src/advanced_servo.cpp)
add_executable(advanced_servo_client src/advanced_servo_client.cpp)
add_executable(high_speed_encoder src/high_speed_encoder.cpp)
add_executable(high_speed_encoder_client src/high_speed_encoder_client.cpp)
add_executable(motor_control_hc src/motor_control_hc.cpp)
add_executable(motor_control_hc_client src/motor_control_hc_client.cpp)
add_executable(motor_control_hc_1065 src/motor_control_hc_1065.cpp)
add_executable(interface_kit src/interface_kit.cpp)
add_executable(interface_kit_client src/interface_kit_client.cpp)
add_executable(accelerometer src/accelerometer.cpp)
add_executable(accelerometer_client src/accelerometer_client.cpp)
add_executable(manager src/manager.cpp)
add_executable(manager_client src/manager_client.cpp)
add_executable(stepper src/stepper.cpp)
add_executable(stepper_client src/stepper_client.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(joystick src/joystick.cpp)
add_executable(lidar src/lidar.cpp)
add_executable(lidar_client src/lidar_client.cpp)
add_executable(joystick_motor_control src/joystick_motor_control.cpp)
add_executable(joystick_servo_control src/joystick_servo_control.cpp)
add_executable(servo_glimpse src/servo_glimpse.cpp)
add_executable(rfid src/rfid.cpp)
add_executable(rfid_client src/rfid_client.cpp)
add_executable(textlcd src/textlcd.cpp)
add_executable(textlcd_client src/textlcd_client.cpp)
add_executable(phsensor src/phsensor.cpp)
add_executable(phsensor_client src/phsensor_client.cpp)
add_executable(spatial src/spatial.cpp)
add_executable(spatial_client src/spatial_client.cpp)
add_executable(led src/led.cpp)
add_executable(led_client src/led_client.cpp)
add_executable(ir src/ir.cpp)
add_executable(ir_client src/ir_client.cpp)

add_dependencies(simple_servo ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(advanced_servo ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(advanced_servo_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(high_speed_encoder ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(high_speed_encoder_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(motor_control_hc ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(motor_control_hc_1065 ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(motor_control_hc_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(interface_kit ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(interface_kit_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(accelerometer ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(accelerometer_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(manager ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(manager_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(stepper ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(stepper_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(odometry ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(joystick ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(lidar ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(lidar_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(joystick_motor_control ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(joystick_servo_control ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(servo_glimpse ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(rfid ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(rfid_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(textlcd ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(textlcd_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(phsensor ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(phsensor_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(spatial ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(spatial_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(led ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(led_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(ir ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(ir_client ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(simple_servo ${catkin_LIBRARIES} phidget21)
target_link_libraries(advanced_servo ${catkin_LIBRARIES} phidget21)
target_link_libraries(advanced_servo_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(high_speed_encoder ${catkin_LIBRARIES} phidget21)
target_link_libraries(high_speed_encoder_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(motor_control_hc ${catkin_LIBRARIES} phidget21)
target_link_libraries(motor_control_hc_1065 ${catkin_LIBRARIES} phidget21)
target_link_libraries(motor_control_hc_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(interface_kit ${catkin_LIBRARIES} phidget21)
target_link_libraries(interface_kit_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(accelerometer ${catkin_LIBRARIES} phidget21)
target_link_libraries(accelerometer_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(manager ${catkin_LIBRARIES} phidget21)
target_link_libraries(manager_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(stepper ${catkin_LIBRARIES} phidget21)
target_link_libraries(stepper_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(odometry ${catkin_LIBRARIES} phidget21)
target_link_libraries(joystick ${catkin_LIBRARIES} phidget21)
target_link_libraries(lidar ${catkin_LIBRARIES} phidget21)
target_link_libraries(lidar_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(joystick_motor_control ${catkin_LIBRARIES} phidget21)
target_link_libraries(joystick_servo_control ${catkin_LIBRARIES} phidget21)
target_link_libraries(servo_glimpse ${catkin_LIBRARIES} phidget21)
target_link_libraries(rfid ${catkin_LIBRARIES} phidget21)
target_link_libraries(rfid_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(textlcd ${catkin_LIBRARIES} phidget21)
target_link_libraries(textlcd_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(phsensor ${catkin_LIBRARIES} phidget21)
target_link_libraries(phsensor_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(spatial ${catkin_LIBRARIES} phidget21)
target_link_libraries(spatial_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(led ${catkin_LIBRARIES} phidget21)
target_link_libraries(led_client ${catkin_LIBRARIES} phidget21)
target_link_libraries(ir ${catkin_LIBRARIES} phidget21)
target_link_libraries(ir_client ${catkin_LIBRARIES} phidget21)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(phidgets_node phidgets_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(phidgets_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS phidgets phidgets_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_phidgets.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
