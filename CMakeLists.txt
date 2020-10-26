cmake_minimum_required(VERSION 3.1.3)
project(humotion)

set(ENV{ROS_LANG_DISABLE} "genjava")

set(ROS_BUILD_TYPE Debug)

################################################################
# check for ROS support:
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    message_generation
    genmsg
    dynamic_reconfigure
)

IF (NOT catkin_FOUND)
  MESSAGE(FATAL_ERROR "Error: could not find ROS middleware!")
ENDIF (NOT catkin_FOUND)

INCLUDE(FindPkgConfig)

##libreflexxes
#IF (libreflexxes_DIR)
#  MESSAGE(STATUS "using libreflexxes_DIR as override ('${libreflexxes_DIR}')")
#  SET(REFLEXXES_PREFIX ${libreflexxes_DIR})
#  SET(REFLEXXES_LIBRARY "ReflexxesTypeII")
#ELSE ()
  PKG_CHECK_MODULES(REFLEXXES REQUIRED libReflexxesTypeII>=1.2.3)
  IF (NOT REFLEXXES_FOUND)
    message(FATAL_ERROR "Error: could not find lib libReflexxesTypeII")
  ENDIF ()
#ENDIF ()


SET(REFLEXXES_LIBRARY_DIRS "${REFLEXXES_PREFIX}/lib")
SET(REFLEXXES_INCLUDE_DIRS "${REFLEXXES_PREFIX}/include")
#resolve to absolute library filename
#or is there an easier way to get the absolute lib filename from pkg config files?!
find_library(REFLEXXES_LIBRARY NAMES ${REFLEXXES_LIBRARIES} HINTS ${REFLEXXES_LIBRARY_DIRS})

MESSAGE(STATUS "using libReflexxesTypeII version ${REFLEXXES_VERSION} from ${REFLEXXES_INCLUDE_DIRS} and ${REFLEXXES_LIBRARY_DIRS}")
MESSAGE(STATUS "will link against ${REFLEXXES_LIBRARY}")

set(CMAKE_CXX_FLAGS "-g -Wall")
add_definitions ("-Wall")

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
        FILES
        gaze.msg
        position_lcr.msg
        mouth.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        humotion
)

# add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/humotion.cfg
)

#
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
    INCLUDE_DIRS include
    LIBRARIES humotion
    #CATKIN_DEPENDS message_runtime
    #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(BEFORE ${Boost_INCLUDE_DIRS} ${REFLEXXES_INCLUDE_DIRS})
include_directories(BEFORE include)

#make sure to use ros messages from current build
include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/include)

#this should be appended:
include_directories(${catkin_INCLUDE_DIRS})

#link_directories (${Boost_LIBRARY_DIRS} ${REFLEXXES_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME}
    src/client/client.cpp
    src/client/middleware.cpp
    src/client/middleware_ros.cpp

    src/server/server.cpp
    src/server/config.cpp
    src/server/middleware.cpp
    src/server/middleware_ros.cpp
    src/server/controller.cpp
    src/server/joint_interface.cpp
    src/server/motion_generator.cpp
    src/server/gaze_motion_generator.cpp
    src/server/reflexxes_motion_generator.cpp
    src/server/mouth_motion_generator.cpp
    src/server/eye_motion_generator.cpp
    src/server/eyelid_motion_generator.cpp
    src/server/eyebrow_motion_generator.cpp
    src/server/neck_motion_generator.cpp

    src/mouth_state.cpp
    src/gaze_state.cpp
    src/timestamp.cpp
    src/timestamped_list.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${REFLEXXES_LIBRARY}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)

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
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark cpp header files for installation
install(DIRECTORY include/humotion/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test-server test/server.cpp)
if(TARGET ${PROJECT_NAME}-test-server)
target_link_libraries(${PROJECT_NAME}-test-server ${PROJECT_NAME})
endif()
catkin_add_gtest(${PROJECT_NAME}-test-client test/client.cpp)
if(TARGET ${PROJECT_NAME}-test-client)
target_link_libraries(${PROJECT_NAME}-test-client ${PROJECT_NAME})
endif()
catkin_add_gtest(${PROJECT_NAME}-test-timestamp test/timestamp.cpp)
if(TARGET ${PROJECT_NAME}-test-timestamp)
target_link_libraries(${PROJECT_NAME}-test-timestamp ${PROJECT_NAME})
endif()

### Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#finally build (some) examples
#add_subdirectory(examples/demo_korg_slider)
#add_subdirectory(examples/meka)
#add_subdirectory(examples/yarp_icub)


#################
## style guide ##
#################
set(ENABLE_CPPLINT 1)
include(${CMAKE_CURRENT_SOURCE_DIR}/stylecheck/CpplintWrapper.cmake)
CPPLINT_RECURSIVE(cpplint_include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/include)
CPPLINT_RECURSIVE(cpplint_src
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_BINARY_DIR}/src)
add_dependencies(${PROJECT_NAME} cpplint_src cpplint_include)

#workaround for qtcreator ide integration. do not remove!
file(GLOB_RECURSE NODE_DUMMY_TARGETS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h *.cfg *.yaml *.xml *.launch)
add_custom_target(_dummy_target SOURCES ${NODE_DUMMY_TARGETS})
