cmake_minimum_required(VERSION 2.8.3)
project(pleurobot_ros_pkg)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs image_transport cv_bridge cmake_modules joy tf)
find_package(Eigen REQUIRED)

catkin_package(  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS opencv
  DEPENDS Eigen
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY Eigen
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(pleurobot_main src/pleurobot_main.cpp src/controller.cpp src/controller_getParameters.cpp src/controller_forces.cpp src/controller_estimation.cpp src/controller_misc.cpp src/controller_snake.cpp src/controller_reflexes_posture.cpp src/controller_torqueCtrl.cpp src/controller_inverseKinematics.cpp src/controller_trajectory.cpp src/controller_trajectory2.cpp src/controller_VMC.cpp src/muscle.cpp src/hill_muscle.cpp src/utils.cpp src/MLP.cpp src/joystick.cpp src/publisher.cpp)
add_library(${PROJECT_NAME} src/controller.cpp)

target_link_libraries(pleurobot_main ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
