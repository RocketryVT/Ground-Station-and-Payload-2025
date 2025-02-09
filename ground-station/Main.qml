// Copyright (C) 2017 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

import QtQuick
import QtLocation
import QtPositioning
import QtQuick.Layouts
import QtQuick.Controls
import QtGraphs
import QtWebView

Window {
    width: Qt.platform.os === "android" ? Screen.width : 1280
    height: Qt.platform.os === "android" ? Screen.height : 720
    visible: true
    title: "Map"

    Plugin {
        id: mapPlugin
        name: "osm"
    }

    MenuBar {
        x: 0
        y: 0
        width: parent.width
        height: 40

        Menu {
            title: qsTr("&File")
            Action { text: qsTr("&New...") }
            Action { text: qsTr("&Open...") }
            Action { text: qsTr("&Save") }
            Action { text: qsTr("Save &As...") }
            MenuSeparator { }
            Action { 
                text: qsTr("&Quit")
                onTriggered: Qt.quit()
            }
        }
        Menu {
            title: qsTr("&Edit")
            Action { text: qsTr("Cu&t") }
            Action { text: qsTr("&Copy") }
            Action { text: qsTr("&Paste") }
        }
        Menu {
            title: qsTr("&Help")
            Action {
                text: qsTr("&About")
                onTriggered: testFunction.myFunction()
            }
        }
    }



    GridLayout {
        x: 0
        y: 40
        width: parent.width
        height: parent.height - 40
        rowSpacing: 0
        columnSpacing: 0
        rows: 2
        columns: 2

        Rectangle {
            id: layout
            // anchors.fill: parent
            // anchors.margins: 10
            Layout.preferredHeight: 400
            Layout.preferredWidth: parent.width/2
            Layout.rowSpan: 2
            Layout.fillHeight: true
            Layout.fillWidth: false
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter

            WebView {
                id: webView
                anchors.fill: parent
                // url: "https://www.example.com"
                // url: ":/web/index.html"
                url: "http://127.0.0.1:5500/web/index.html"
                settings.javaScriptEnabled: true
            }
        }

        

        // Map {
        //     id: map
        //     // anchors.fill: parent
        //     // width: 400
        //     // height: 300
        //     Layout.preferredHeight: 400
        //     Layout.preferredWidth: parent.width/2
        //     plugin: mapPlugin
        //     center: QtPositioning.coordinate(59.91, 10.75) // Oslo
        //     zoomLevel: 14
        //     property geoCoordinate startCentroid
        //     Layout.rowSpan: 2
        //     Layout.fillHeight: true
        //     Layout.fillWidth: false
        //     Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
        //     // anchors.fill: parent
        //     // anchors.leftMargin: 16
        //     // anchors.rightMargin: 667
        //     // anchors.topMargin: 16
        //     // anchors.bottomMargin: 305
        //     maximumTilt: 0.6
        //     tilt: 0
        //     antialiasing: true

        //     PinchHandler {
        //         id: pinch
        //         target: null
        //         onActiveChanged: if (active) {
        //                              map.startCentroid = map.toCoordinate(pinch.centroid.position, false)
        //                          }
        //         onScaleChanged: (delta) => {
        //                             map.zoomLevel += Math.log2(delta)
        //                             map.alignCoordinateToPoint(map.startCentroid, pinch.centroid.position)
        //                         }
        //         onRotationChanged: (delta) => {
        //                                map.bearing -= delta
        //                                map.alignCoordinateToPoint(map.startCentroid, pinch.centroid.position)
        //                            }
        //         grabPermissions: PointerHandler.TakeOverForbidden
        //     }
        //     WheelHandler {
        //         id: wheel
        //         // workaround for QTBUG-87646 / QTBUG-112394 / QTBUG-112432:
        //         // Magic Mouse pretends to be a trackpad but doesn't work with PinchHandler
        //         // and we don't yet distinguish mice and trackpads on Wayland either
        //         acceptedDevices: Qt.platform.pluginName === "cocoa" || Qt.platform.pluginName === "wayland"
        //                          ? PointerDevice.Mouse | PointerDevice.TouchPad
        //                          : PointerDevice.Mouse
        //         rotationScale: 1/120
        //         property: "zoomLevel"
        //     }
        //     DragHandler {
        //         id: drag
        //         target: null
        //         onTranslationChanged: (delta) => map.pan(-delta.x, -delta.y)
        //     }
        //     Shortcut {
        //         enabled: map.zoomLevel < map.maximumZoomLevel
        //         sequence: StandardKey.ZoomIn
        //         onActivated: map.zoomLevel = Math.round(map.zoomLevel + 1)
        //     }
        //     Shortcut {
        //         enabled: map.zoomLevel > map.minimumZoomLevel
        //         sequence: StandardKey.ZoomOut
        //         onActivated: map.zoomLevel = Math.round(map.zoomLevel - 1)
        //     }
        // }

        Rectangle {
            id: rectangle1
            Layout.preferredWidth: parent.width/2
            Layout.preferredHeight: 165
            color: "#2718ff"
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            transformOrigin: Item.Center
            Layout.fillHeight: true
            Layout.fillWidth: true

            Text {
                text: "Value from C++: " + counter.counter
                anchors.centerIn: parent
            }
        }

        Rectangle {
            id: rectangle
            Layout.preferredWidth: parent.width/2
            Layout.preferredHeight: 165
            color: "#ff148d"
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            Layout.fillHeight: true
            Layout.fillWidth: true
        }

        GraphsView {
            id: line
            Layout.fillWidth: true
            Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
            Layout.columnSpan: 2
            Layout.preferredHeight: 263
            Layout.preferredWidth: parent.width

            axisX: ValueAxis {
                id: axisX
                min: 0
                max: 60
            }
            
            axisY: ValueAxis {
                id: axisY
                min: 0
                max: 100
            }

            LineSeries {
                id: lineSeries
                color: "blue"
            }
        }

        // Rectangle {
        //     id: rectangle2
        //     color: "#781eff"
        //     Layout.fillWidth: true
        //     Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
        //     Layout.columnSpan: 2
        //     Layout.preferredHeight: 263
        //     Layout.preferredWidth: parent.width
        // }


    }

    Connections {
        target: counter
        function onNewDataPoint(time, value) {
            lineSeries.append(time, value)
            console.log("New data point added:", time, value)
            if (time > axisX.max) {
                axisX.max = time
            }
            if (value > axisY.max) {
                axisY.max = value
            }
        }
    }
}
