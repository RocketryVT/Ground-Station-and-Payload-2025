# Generated from QtQMLTest.pro.

cmake_minimum_required(VERSION 3.16)
project(QtQMLTest LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(NOT DEFINED INSTALL_EXAMPLESDIR)
  set(INSTALL_EXAMPLESDIR "examples")
endif()

set(INSTALL_EXAMPLEDIR "${INSTALL_EXAMPLESDIR}/location/QtQMLTest")

find_package(Qt6 COMPONENTS Core)
find_package(Qt6 COMPONENTS Gui)
find_package(Qt6 COMPONENTS Location)
find_package(Qt6 COMPONENTS Qml)

qt_add_qml_module(Counter
    URI com.example.counter
    VERSION 1.0
    SOURCES
        Counter.h
        Counter.cpp
)

qt_add_executable(QtQMLTest WIN32 MACOSX_BUNDLE
    main.cpp
        Counter.cpp
        Counter.h
)
target_link_libraries(QtQMLTest PUBLIC
    Qt::Core
    Qt::Gui
    Qt::Location
    Qt::Qml
)


# Resources:
set(qml_resource_files
    "main.qml"
)

qt6_add_resources(QtQMLTest "qml"
    PREFIX
        "/"
    FILES
        ${qml_resource_files}
)

install(TARGETS QtQMLTest
    RUNTIME DESTINATION "${INSTALL_EXAMPLEDIR}"
    BUNDLE DESTINATION "${INSTALL_EXAMPLEDIR}"
    LIBRARY DESTINATION "${INSTALL_EXAMPLEDIR}"
)
