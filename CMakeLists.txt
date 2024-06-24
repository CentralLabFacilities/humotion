cmake_minimum_required(VERSION 3.1.3)
project(humotion)

find_program(CLANG_TIDY NAMES clang-tidy clang-tidy-10 clang-tidy-9 clang-tidy-8 clang-tidy-7 clang-tidy-6)
if (CLANG_TIDY)
  set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY}")
endif()

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

find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(REFLEXXES REQUIRED libReflexxesTypeII>=1.2.3)
SET(REFLEXXES_LIBRARY_DIRS "${REFLEXXES_PREFIX}/lib")
SET(REFLEXXES_INCLUDE_DIRS "${REFLEXXES_PREFIX}/include")
# Resolve to absolute library filename
find_library(REFLEXXES_LIBRARY NAMES ${REFLEXXES_LIBRARIES} HINTS ${REFLEXXES_LIBRARY_DIRS})

MESSAGE(STATUS "using libReflexxesTypeII version ${REFLEXXES_VERSION} from ${REFLEXXES_INCLUDE_DIRS} and ${REFLEXXES_LIBRARY_DIRS}")
MESSAGE(STATUS "will link against ${REFLEXXES_LIBRARY}")

add_compile_options("-Wall")

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
include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${REFLEXXES_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

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


#workaround for qtcreator ide integration. do not remove!
file(GLOB_RECURSE NODE_DUMMY_TARGETS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h *.cfg *.yaml *.xml *.launch)
add_custom_target(_dummy_target SOURCES ${NODE_DUMMY_TARGETS})
