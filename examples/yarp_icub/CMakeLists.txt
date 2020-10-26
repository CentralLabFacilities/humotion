PROJECT(humotion_yarp_icub)
cmake_minimum_required(VERSION 2.8)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(humotion)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# add include directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${humotion_INCLUDE_DIRS} include/)
link_directories(${humotion_LIBRARY_DIRS})

# add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
SET(MAIN icub_humotion_server)

file(GLOB DUMMY_HEADER_LIST RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.h)

ADD_EXECUTABLE(${MAIN} src/main.cpp src/icub_jointinterface.cpp src/icub_faceinterface.cpp src/icub_data_receiver.cpp ${DUMMY_HEADER_LIST})

# we now add the YARP and iCub libraries to our project.
TARGET_LINK_LIBRARIES(${MAIN} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${humotion_LIBRARIES})
set_property(TARGET ${MAIN} PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)

INSTALL(TARGETS ${MAIN} DESTINATION bin)


#################
## style guide ##
#################
SET(CPPLINT_CMAKE ${CMAKE_CURRENT_SOURCE_DIR}/../../stylecheck/CpplintWrapper.cmake)

if(EXISTS "${CPPLINT_CMAKE}")
  set(ENABLE_CPPLINT 1)
  include(${CPPLINT_CMAKE})
  CPPLINT_RECURSIVE(cpplint_include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/include)
  CPPLINT_RECURSIVE(cpplint_src
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}/src)
  add_dependencies(${MAIN} cpplint_src cpplint_include)
else ()
  message(WARNING "Could not find cpplint cmake file. Will not do code stylecheck!")
endif ()
#workaround for qtcreator ide integration. do not remove!
file(GLOB_RECURSE NODE_DUMMY_TARGETS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h *.cfg *.yaml *.xml *.launch)
add_custom_target(_dummy_target SOURCES ${NODE_DUMMY_TARGETS})
