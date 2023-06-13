import sys
import os
import signal
import pygame
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
import subprocess
import ament_index_python
import io

try:
    import folium
    from PySide6.QtWidgets import (
        QApplication,
        QLabel,
        QMainWindow,
        QWidget,
        QHBoxLayout,
        QGraphicsScene,
        QGraphicsView,
        QGraphicsProxyWidget,
        QPushButton,
    )
    from PySide6.QtCore import QThread, QObject, Signal, Qt, QUrl, QByteArray, QRect
    from PySide6.QtGui import QPalette, QColor
    from PySide6.QtWebEngineWidgets import *
    from PySide6.QtWebEngineCore import *
    from qt_material import apply_stylesheet
except ImportError as e:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "PySide6"])
    subprocess.check_call([sys.executable, "-m", "pip", "install", "folium"])
    subprocess.check_call([sys.executable, "-m", "pip", "install", "qt-material"])
    import folium
    from PySide6.QtWidgets import (
        QApplication,
        QLabel,
        QMainWindow,
        QWidget,
        QHBoxLayout,
        QGraphicsScene,
        QGraphicsView,
        QGraphicsProxyWidget,
        QPushButton,
    )
    from PySide6.QtCore import QThread, QObject, Signal, Qt, QUrl, QByteArray, QRect
    from PySide6.QtGui import QPalette, QColor
    from PySide6.QtWebEngineWidgets import *
    from PySide6.QtWebEngineCore import *
    from qt_material import apply_stylesheet

from multiprocessing import Process
from multiprocessing.connection import Listener
from multiprocessing.connection import Connection, Client
import threading
import time
from enum import Enum

package_name = "sailbot_test"


def get_package_path(package_name):
    package_path = ament_index_python.get_package_share_directory(package_name)
    return package_path


class BoatDataType(Enum):
    WIND = 1
    NAVSAT = 2


class BoatData:
    def __init__(self, type: BoatDataType) -> None:
        self.type = type

    type: BoatDataType


class WindData(BoatData):
    def __init__(self) -> None:
        super().__init__(BoatDataType.WIND)

    wind: Vector3


class NavSatData(BoatData):
    def __init__(self) -> None:
        super().__init__(BoatDataType.NAVSAT)

    navSat: NavSatFix


class Color(QWidget):
    def __init__(self, color):
        super(Color, self).__init__()
        self.setAutoFillBackground(True)

        palette = self.palette()
        palette.setColor(QPalette.ColorRole.Window, QColor(color))
        self.setPalette(palette)


class MainWindow(QMainWindow):
    graphics_view: QGraphicsView
    graphics_scene: QGraphicsScene

    map_view: QWebEngineView
    map_proxy_widget: QGraphicsProxyWidget

    wind_hbox: QHBoxLayout
    wind_label: QLabel
    wind_label_x: QLabel
    wind_label_y: QLabel
    wind_label_z: QLabel
    wind_widget = QWidget

    map_view: QWebEngineView

    current_navsat: NavSatFix
    current_navsat_lock: threading.Lock
    def __init__(self):
        super().__init__()

        #locks
        self.current_navsat_lock = threading.Lock()

        # Create the graphics view and scene
        self.graphics_view = QGraphicsView()
        self.graphics_view.setStyleSheet("border: 1px solid black;")
        self.graphics_scene = QGraphicsScene()
        self.graphics_scene.setStyle
        self.graphics_view.setScene(self.graphics_scene)
        self.graphics_view.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        self.graphics_view.setVerticalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )

        # wind
        self.wind_hbox = QHBoxLayout()
        self.wind_label = QLabel()
        self.wind_label.setText("Wind: ")
        self.wind_label_x = QLabel()
        self.wind_label_x.setFixedWidth(50)
        self.wind_label_x.setFixedHeight(20)
        self.wind_label_x.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.wind_label_x.setStyleSheet("border: 1px solid black;")
        self.wind_label_y = QLabel()
        self.wind_label_y.setFixedWidth(50)
        self.wind_label_y.setFixedHeight(20)
        self.wind_label_y.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.wind_label_y.setStyleSheet("border: 1px solid black;")
        self.wind_label_z = QLabel()
        self.wind_label_z.setFixedWidth(50)
        self.wind_label_z.setFixedHeight(20)
        self.wind_label_z.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.wind_label_z.setStyleSheet("border: 1px solid black;")
        self.wind_hbox.addWidget(self.wind_label)
        self.wind_hbox.addWidget(QLabel("X:"))
        self.wind_hbox.addWidget(self.wind_label_x)
        self.wind_hbox.addWidget(QLabel("Y:"))
        self.wind_hbox.addWidget(self.wind_label_y)
        self.wind_hbox.addWidget(QLabel("Z:"))
        self.wind_hbox.addWidget(self.wind_label_z)
        self.wind_widget = QWidget()
        self.wind_widget.setLayout(self.wind_hbox)

        wind_proxy_widget = QGraphicsProxyWidget()
        wind_proxy_widget.setWidget(self.wind_widget)
        # layout.addWidget(self.wind_widget)

        # map
        self.map_view = QWebEngineView()
        # settings = self.map_view.settings()
        # settings.setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)
        # package_path = get_package_path(package_name)
        coordinate = (37.8199286, -122.4782551)
        self.map = folium.Map(
            tiles="OpenStreetMap", zoom_start=13, max_zoom=19, location=coordinate
        )
        # save map data to data object
        data = io.BytesIO()
        self.map.save(data, close_file=False)
        with open("/home/matthew/Desktop/saved_map.html", "wb") as file:
            self.map.save(file)
        self.map_view.setHtml(data.getvalue().decode())
        self.map_proxy_widget = QGraphicsProxyWidget()
        self.map_proxy_widget.setWidget(self.map_view)
        self.has_updated_map = False

        center_btn = QPushButton("Center on boat")
        center_btn.clicked.connect(self.center_btn_click)
        center_btn_proxy_widget = QGraphicsProxyWidget()
        center_btn_proxy_widget.setWidget(center_btn)

        self.graphics_scene.addItem(self.map_proxy_widget)
        self.graphics_scene.addItem(wind_proxy_widget)
        self.graphics_scene.addItem(center_btn_proxy_widget)
        center_btn.move(20, 90)
        self.wind_widget.move(70, 20)

        # set correct order
        # self.wind_widget.raise_()

        # Set the central widget of the Window. Widget will expand
        # to take up all the space in the window by default.
        self.setCentralWidget(self.graphics_view)
        self.setWindowTitle("Sailbot Control")

    def resizeEvent(self, event):
        # Update the size of the graphics view
        size = event.size()

        self.map_proxy_widget.resize(size.width(), size.height())
        self.map_proxy_widget.update()
        # Call the base class resizeEvent
        super().resizeEvent(event)

    def closeEvent(self, event):
        super().closeEvent(event)

    def update_boat_data(self, boat_data: BoatData):
        match boat_data.type:
            case BoatDataType.WIND:
                self.update_wind_data(boat_data)
            case BoatDataType.NAVSAT:
                self.update_navsat_data(boat_data)

    def create_icon_and_center_map(self, boat_data: NavSatData):
        move_command = (
            self.map.get_name()
            + ".panTo(new L.LatLng("
            + str(boat_data.navSat.latitude)
            + ", "
            + str(boat_data.navSat.longitude)
            + "));"
        )

        try:
            self.map_view.page().runJavaScript(move_command)
            self.has_updated_map = True
        except:
            # map sometimes loads too slow
            return
        # TODO: Find a way to make this icon local, instead of sourcing it from a website of unknown reliability
        create_marker_command = (
            """var LeafIcon = L.Icon.extend({
		options: {
			iconSize:     [32, 32],
			iconAnchor:   [16, 5]
		}
        });"""
            + "var boat = new LeafIcon({iconUrl: 'https://cdn2.iconfinder.com/data/icons/circle-icons-1/64/sailboat-512.png'});\n"
        )
        create_marker_command += (
            "var boatMarker = L.marker(["
            + str(boat_data.navSat.latitude)
            + ", "
            + str(boat_data.navSat.longitude)
            + "], {icon: boat, draggable: false}).addTo("
            + self.map.get_name()
            + ");"
        )
        print(create_marker_command)
        self.map_view.page().runJavaScript(create_marker_command)
    
    def center_btn_click(self):
        with self.current_navsat_lock:
            move_command = (
                self.map.get_name()
                + ".panTo(new L.LatLng("
                + str(self.current_navsat.latitude)
                + ", "
                + str(self.current_navsat.longitude)
                + "));"
            )
        try:
            self.map_view.page().runJavaScript(move_command)
            self.has_updated_map = True
        except:
            # map sometimes loads too slow
            return

    def update_wind_data(self, boat_data: WindData):
        self.wind_label_x.setText("{0:.2f}".format(boat_data.wind.x))
        self.wind_label_y.setText("{0:.2f}".format(boat_data.wind.y))
        self.wind_label_z.setText("{0:.2f}".format(boat_data.wind.z))

    def update_navsat_data(self, boat_data: NavSatData):
        with self.current_navsat_lock:
            self.current_navsat = boat_data.navSat
        if not self.has_updated_map:
            self.create_icon_and_center_map(boat_data)
        else:
            move_marker_command = (
                "var newPosition = L.latLng("
                + str(boat_data.navSat.latitude)
                + ", "
                + str(boat_data.navSat.longitude)
                + "); boatMarker.setLatLng(newPosition);"
            )
            self.map_view.page().runJavaScript(move_marker_command)


def recieve_messages(listener: Listener):
    conn = listener.accept()
    while True:
        msg = conn.recv()
        time.sleep(1)
        # do something with msg


class RecieveMessages(QObject):
    listener: Listener
    progress = Signal(BoatData)

    def __init__(self, listener: Listener):
        super().__init__()
        self.listener = listener

    def run(self):
        conn = self.listener.accept()
        while True:
            msg = conn.recv()
            self.progress.emit(msg)


def CreateUI():
    app = QApplication(sys.argv)
    window = MainWindow()

    # Connect IPC
    address = ("localhost", 6000)
    listener = Listener(address, authkey=b"password")
    thread = QThread()
    worker = RecieveMessages(listener)
    worker.moveToThread(thread)
    worker.progress.connect(window.update_boat_data)
    thread.started.connect(worker.run)
    thread.start()
    apply_stylesheet(app, theme='light_cyan_500.xml')
    window.show()
    app.exec()

    # exit
    listener.close()


class SailbotUI(Node):
    ui_proc: Process
    ui_conn: Connection

    def __init__(self):
        super().__init__("sailbot_ui")
        self.anemometer_subscriber = self.create_subscription(
            Vector3,
            "/world/waves/model/rs750/link/base_link/sensor/anemometer/anemometer",
            self.anemometer_callback,
            0,
        )
        self.anemometer_subscriber = self.create_subscription(
            NavSatFix,
            "/world/waves/model/rs750/link/base_link/sensor/navsat_sensor/navsat",
            self.navsat_callback,
            0,
        )
        self.main_sail_publisher_ = self.create_publisher(
            Float64, "/main_sail_joint/cmd_pos", 10
        )
        self.rudder_publisher_ = self.create_publisher(
            Float64, "/rudder_joint/cmd_pos", 10
        )
        # self.main_ui = UI()
        self.ui_proc = Process(target=CreateUI)
        self.ui_proc.start()
        # Setup IPC
        address = ("localhost", 6000)
        while True:
            try:
                self.ui_conn = Client(address, authkey=b"password")
            except:
                self.get_logger().warn("Could not connect to UI, retrying...")
                time.sleep(1)
                pass
            else:
                self.get_logger().info("Connected to UI")
                break

    def anemometer_callback(self, msg: Vector3):
        boat_data = WindData()
        boat_data.wind = msg
        self.ui_conn.send(boat_data)

    def navsat_callback(self, msg: NavSatFix):
        boat_data = NavSatData()
        boat_data.navSat = msg
        self.ui_conn.send(boat_data)


def main(args=None):
    rclpy.init(args=args)
    sailbot_ui = SailbotUI()
    rclpy.spin(sailbot_ui)
    sailbot_ui.ui_proc.kill()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sailbot_ui.destroy_node()
    rclpy.shutdown()


# do not use this
if __name__ == "__main__":
    main()
