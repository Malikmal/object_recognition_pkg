cmake_minimum_required(VERSION 3.0.2)
project(object_recognition_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_detection_recognition
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/object_detection_recognition.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/object_detection_recognition_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_detection_recognition.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


# -------- MY CODE ---------- #

# start of opencv
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)


add_message_files(
  FILES
  trackbar.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pkg
   CATKIN_DEPENDS roscpp std_msgs message_runtime 
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(	
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)
#end of opencv




# Find PointCLoud Libraries
find_package(PCL 1.2 REQUIRED)

#find_package(realsense2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(HDF5 REQUIRED)
# find_package(FLANN REQUIRED)


include_directories(SYSTEM
  ${HDF5_INCLUDE_DIR}
)

include_directories(/Fann/include)
link_directories(/Fann/lib)

# main node
add_executable(main src/main.cpp)
target_link_libraries(main 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    fann
)

# main_trainning node
add_executable(main_trainning src/main_trainning.cpp)
target_link_libraries(main_trainning 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    fann
)


# main_trainning_testing node
add_executable(main_trainning_testing src/main_trainning_testing.cpp)
target_link_libraries(main_trainning_testing 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    fann
)

# # segmentation node
# add_executable(segmentation src/segmentation.cpp)
# target_link_libraries(segmentation 
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
# )


# # extract_feature node
# add_executable(extract_feature src/extract_feature.cpp)
# target_link_libraries(extract_feature 
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${Boost_LIBRARIES}
#     # FLANN::FLANN ${HDF5_LIBRARIES}
    
# )

# # learnfann node
# add_executable(learnfann src/learnfann.cpp)
# target_link_libraries(learnfann 
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
#     ${Boost_LIBRARIES}
#     fann
#     # FLANN::FLANN ${HDF5_LIBRARIES}
    
# )


# SET(PLATFORM_SPECIFIC_LIBS "-lpthread")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -pthread")

# # try_kinect node
# add_executable(try_kinect src/try_kinect.cpp)
# target_link_libraries(try_kinect 
#     # ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
#     # ${Boost_LIBRARIES}
#     # fann
#     # FLANN::FLANN ${HDF5_LIBRARIES}
    
# )



# # opencv node
# add_executable(opencv src/opencv.cpp)
# target_link_libraries(opencv 
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
#     # ${Boost_LIBRARIES}
#     # fann
#     # FLANN::FLANN ${HDF5_LIBRARIES}
    
# )



# # testviz node
# add_executable(testviz src/testviz.cpp)
# target_link_libraries(testviz 
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
# )






# main_trainning_own_dataset node
add_executable(main_trainning_own_dataset src/main_trainning_own_dataset.cpp)
target_link_libraries(main_trainning_own_dataset 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    fann
)


# main_camera node
add_executable(main_camera src/main_camera.cpp)
target_link_libraries(main_camera 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    fann
    ${DEPENDENCIES}
    ${realsense2_LIBRARY}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${SPECIAL_OS_LIBS}
    ${OpenCV_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)


# camera_node node
add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    fann
    ${DEPENDENCIES}
    ${realsense2_LIBRARY}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${SPECIAL_OS_LIBS}
    ${OpenCV_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)



# preprocesing_node node
add_executable(preprocesing_node src/preprocesing_node.cpp)
target_link_libraries(preprocesing_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    fann
    ${DEPENDENCIES}
    ${realsense2_LIBRARY}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${SPECIAL_OS_LIBS}
    ${OpenCV_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
add_dependencies(preprocesing_node object_recognition_pkg_generate_messages_cpp)



# trackbar_node node
add_executable(trackbar_node src/trackbar_node.cpp)
target_link_libraries(trackbar_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    fann
    ${DEPENDENCIES}
    ${realsense2_LIBRARY}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${SPECIAL_OS_LIBS}
    ${OpenCV_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)
add_dependencies(trackbar_node object_recognition_pkg_generate_messages_cpp)