SET( PROJECT_NAME demo_korg_slider )
PROJECT( ${PROJECT_NAME} )
# set (CMAKE_VERBOSE_MAKEFILE true)
#profiling
#set(CMAKE_CXX_FLAGS "-Wall -g -pg")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /homes/sschulz/src/ros_workspace/install/share/humotion/cmake/)
# set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /homes/sschulz/src/ros_workspace/install/lib/pkg_config)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
INCLUDE(FindPkgConfig)

# search for Boost version 1.34
find_package( Boost 1.40 COMPONENTS thread program_options system filesystem regex )
link_directories ( ${Boost_LIBRARY_DIRS}  )
include_directories ( ${Boost_INCLUDE_DIRS} )
# message(${Boost_LIBRARIES})



SET(PREFIX ${CMAKE_INSTALL_PREFIX})
SET(BINDIR "${PREFIX}/bin")
SET(INCLUDEDIR "${PREFIX}/include")
SET(MANDIR "${PREFIX}/man")
SET(LIBDIR "${PREFIX}/lib")
SET(DATADIR "${PREFIX}/share/${NAME}")

find_package(humotion 0.0.1)
IF (humotion_FOUND)
	message("> using humotion includes from " ${humotion_INCLUDE_DIRS})
ELSE (humotion_FOUND)
	message(FATAL_ERROR "> error: can not find libhumotion")
endif (humotion_FOUND)

SET(SRC_FILES
	korg_input.cpp
)

LINK_DIRECTORIES(${LIBDIR} ${humotion_LIBRARY_DIRS} )
INCLUDE_DIRECTORIES( ${humotion_INCLUDE_DIRS} )

ADD_EXECUTABLE( ${PROJECT_NAME} main.cpp ${SRC_FILES})
TARGET_LINK_LIBRARIES(  ${PROJECT_NAME} ${Boost_LIBRARIES} ${Boost_LIBS}  ${humotion_LIBRARIES} portmidi porttime) 


#FILE(GLOB conf_files "${CMAKE_CURRENT_SOURCE_DIR}/*.mcfg")
#install (FILES ${conf_files} DESTINATION etc/xsc2)

install (TARGETS ${PROJECT_NAME} DESTINATION bin )
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

