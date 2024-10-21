# Generated from QtQMLTest.pro.

cmake_minimum_required(VERSION 3.16)
project(GroundStation LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(INSTALL_DIR "./build")

find_package(Qt6 COMPONENTS Core)
find_package(Qt6 COMPONENTS Gui)
find_package(Qt6 COMPONENTS Location)
find_package(Qt6 COMPONENTS Positioning)
find_package(Qt6 COMPONENTS Qml)
find_package(Qt6 COMPONENTS Quick)
find_package(Qt6 COMPONENTS Widgets)
find_package(Qt6 COMPONENTS Graphs)

qt_add_executable(GroundStation WIN32 MACOSX_BUNDLE
    main.cpp
    Counter.h
    TestFunction.h
)
target_link_libraries(GroundStation PUBLIC
    Qt::Core
    Qt::Gui
    Qt::Location
    Qt::Positioning
    Qt::Qml
    Qt::Quick
    Qt::Widgets
    Qt::Graphs
)


# Resources:
set(qml_resource_files
    "main.qml"
)

qt6_add_resources(GroundStation "qml"
    PREFIX
        "/"
    FILES
        ${qml_resource_files}
)

install(TARGETS GroundStation
    RUNTIME DESTINATION "${INSTALL_DIR}"
    BUNDLE DESTINATION "${INSTALL_DIR}"
    LIBRARY DESTINATION "${INSTALL_DIR}"
)
