cmake_minimum_required(VERSION 2.8.3)

project(arena_camera)

#
# ARENA SDK
#

# Adds:
# - ${_arena_sdk_ROOT}
# - ${arena_sdk_INCLUDE_DIRS} and ${arena_sdk_INCLUDES}
# - ${arena_sdk_LIBRARIES_DIRS} and ${arena_sdk_LIBS}
# - ${arena_sdk_FOUND}

find_package(arena_sdk QUIET)
if (NOT ${arena_sdk_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Findarena_sdk.cmake")
endif()

#
# CATKIN
#

set(CATKIN_COMPONENTS
    actionlib
    camera_control_msgs
    camera_info_manager
    cv_bridge
    diagnostic_updater
    image_geometry
    image_transport
    roscpp
    roslint
    roslaunch 
    sensor_msgs
    cav_msgs
)

# Find catkin macros and libraries.
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages.
find_package(
    catkin 
        REQUIRED
    COMPONENTS 
        ${CATKIN_COMPONENTS}
)

# This macro ensures modules and global scripts declared therein get installed.
# - Enable Python module support
# - See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# http://wiki.ros.org/catkin/CMakeLists.txt#catkin_package.28.29
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        ${CATKIN_COMPONENTS}
)

#
# ROS CHECKS
#

set(ROSLINT_CPP_OPTS
   "--extensions=cpp,h,hpp" "--filter=-runtime/references,-readability/todo,-build/include_what_you_use"
)

# Check C/C++ static checking. More information at http://wiki.ros.org/roslint.
# No files means all files.
roslint_cpp()

# Closest doc (http://wiki.ros.org/rosbuild/CMakeLists#rosbuild_add_roslaunch_check).
# More https://answers.ros.org/question/200359/purpose-of-roslaunch_add_file_check/.
roslaunch_add_file_check(launch)

#  Flags for all C++ targets.
add_definitions("-std=gnu++11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#
# ARENA_CAMERA_NODE
#

set(arena_camera_node_name arena_camera_node)

# Declare executables.
# Needed ".cpp"s.
add_executable(
    ${arena_camera_node_name}
        src/arena_camera_node.cpp
        src/arena_camera_parameter.cpp
        src/arena_camera.cpp
        src/encoding_conversions.cpp
        src/main.cpp
)

# Needed ".h"s.
target_include_directories(
    ${arena_camera_node_name}
        PRIVATE ${catkin_INCLUDE_DIRS} 
        PRIVATE ${arena_sdk_INCLUDE_DIRS}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Needed ".so"s.
target_link_libraries(
    ${arena_camera_node_name}
    ${catkin_LIBRARIES}
    ${arena_sdk_LIBRARIES}
)

add_dependencies(
    ${arena_camera_node_name}
    ${catkin_EXPORTED_TARGETS} 
)

# target_link_directories NOT NEEDED as arena_sdk_LIBRARIES is 
# defined with absolute paths.

#
# WRITE_DEVICE_USER_ID_TO_CAMERA
#

set(write_device_user_id_to_camera write_device_user_id_to_camera)

# Declare executables
# Needed ".cpp"s.
add_executable(
    ${write_device_user_id_to_camera}
        src/write_device_user_id_to_camera.cpp
)

# Needed ".h"s.
target_include_directories(
    ${write_device_user_id_to_camera}
        PRIVATE ${arena_sdk_INCLUDE_DIRS}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Needed ".so"s.
target_link_libraries(
    ${write_device_user_id_to_camera}
        ${arena_sdk_LIBRARIES}
)


# target_link_directories NOT NEEDED as arena_sdk_LIBRARIES is 
# define with absolute paths

install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

install(
    DIRECTORY
        config/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

install(
    PROGRAMS
        scripts/file_sequencer.py
        scripts/grab_and_save_image_action_server.py
        scripts/result_bag_to_action.py
        scripts/sequence_to_file.py
        scripts/toggle_camera
        scripts/triggered_image_topic.py
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    TARGETS
        ${arena_camera_node_name}
        ${write_device_user_id_to_camera}
    LIBRARY DESTINATION
        ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
        include/${PROJECT_NAME}/
    DESTINATION
        ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN
        "*.h"
    PATTERN "internal" EXCLUDE
)

#
# Testing
#

# All Jenkins-Tests are now in the arena_camera_tests package.

#
# Qt Creator
#

# Entry for QtCreator to show all files
file(GLOB children ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(child ${children})
    if(IS_DIRECTORY ${child})
        file(GLOB_RECURSE dir_files "${child}/*")
        list(APPEND ${PROJECT_NAME}_extra_files ${dir_files})
    endif()
endforeach()