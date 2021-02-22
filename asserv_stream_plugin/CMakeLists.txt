cmake_minimum_required(VERSION 3.10)

PROJECT(plotjuggler_asservStream)



#include_directories( )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS
    Core
    Widgets
    Concurrent
    Xml
    Svg
    WebSockets
    SerialPort)

set( QT_LINK_LIBRARIES
    Qt5::Core
    Qt5::Widgets
    Qt5::Concurrent
    Qt5::Xml
    Qt5::Svg
    Qt5::WebSockets
    Qt5::SerialPort)
    
include_directories(
    3rdparty
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Concurrent_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5WebSockets_INCLUDE_DIRS}
    ${Qt5Svg_INCLUDE_DIRS}
    ${Qt5SerialPort_INCLUDE_DIRS}
    ./
    ../PlotJuggler/include
     ../PlotJuggler/common
)


add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)


QT5_WRAP_UI ( UI_SRC asservstreamcontrolpanel.ui )

SET( SRC
    AsservStream_uartDecoder.cpp
    AsservStream.cpp
    AsservStreamControlPanel.cpp
    ../PlotJuggler/include/PlotJuggler/datastreamer_base.h
    )
    
    
add_library(AsservStream SHARED ${SRC} ${UI_SRC} )
target_link_libraries(AsservStream  ${Qt5Widgets_LIBRARIES} ${Qt5Xml_LIBRARIES} ${Qt5SerialPort_LIBRARIES})
target_compile_definitions(AsservStream PRIVATE PLUGIN_NAME="Esial Robotik asserv stream")

if(COMPILING_WITH_CATKIN)
    install(TARGETS AsservStream
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
else()
    install(TARGETS AsservStream DESTINATION bin  )
    
endif()
