PROJECT(meka_humotion_server)
cmake_minimum_required(VERSION 2.8)
SET(MAIN ${PROJECT_NAME})

set(ENV{ROS_LANG_DISABLE} "genjava")
set(ROS_BUILD_TYPE Debug)

################################################################
# check for ROS support:
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs message_generation genmsg)
IF (catkin_FOUND)
    set(ROS_FOUND 1)
    message(STATUS "ROS Support is ON")
    add_definitions(-DROS_SUPPORT=1)
ENDIF (catkin_FOUND)

IF (NOT catkin_FOUND)
  message(FATAL_ERROR "Error: could not find ROS!")
ENDIF ()

FIND_PACKAGE(Boost REQUIRED COMPONENTS system thread)
FIND_PACKAGE(humotion REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#add_message_files(
#    FILES
#)

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#)

#hack to allow sub dir calls work

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES humotion
    CATKIN_DEPENDS m3meka_msgs
    #DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS})

# add include directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${humotion_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include/)
link_directories(${Boost_LIBRARY_DIRS} ${humotion_LIBRARY_DIRS})

# add required linker flags

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
file(GLOB DUMMY_HEADER_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.h)

#add_dependencies(${MAIN} ${catkin_EXPORTED_TARGETS})
ADD_EXECUTABLE(${MAIN} src/main.cpp src/mekajointinterface.cpp ${DUMMY_HEADER_LIST})

TARGET_LINK_LIBRARIES(${MAIN} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${humotion_LIBRARIES})
set_property(TARGET ${MAIN} PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)


#INSTALL(TARGETS ${MAIN} DESTINATION bin)
install(TARGETS ${MAIN}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION bin 
)

