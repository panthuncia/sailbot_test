import sys
import typing
from PyQt6.QtCore import QObject
import pygame
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import subprocess

try:
    from PyQt6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDateEdit,
        QDateTimeEdit,
        QDial,
        QDoubleSpinBox,
        QFontComboBox,
        QLabel,
        QLCDNumber,
        QLineEdit,
        QMainWindow,
        QProgressBar,
        QPushButton,
        QRadioButton,
        QSlider,
        QSpinBox,
        QTimeEdit,
        QVBoxLayout,
        QWidget,
    )
    from PyQt6.QtCore import QThread, QObject, pyqtSignal
except ImportError as e:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "PyQt6"])
    from PyQt6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDateEdit,
        QDateTimeEdit,
        QDial,
        QDoubleSpinBox,
        QFontComboBox,
        QLabel,
        QLCDNumber,
        QLineEdit,
        QMainWindow,
        QProgressBar,
        QPushButton,
        QRadioButton,
        QSlider,
        QSpinBox,
        QTimeEdit,
        QVBoxLayout,
        QWidget,
    )
    from PyQt6.QtCore import QThread, QObject, pyqtSignal
from multiprocessing import Process
from multiprocessing.connection import Listener
from multiprocessing.connection import Connection, Client
import time

class BoatData():
    wind: Vector3

class MainWindow(QMainWindow):
    wind_label: QLabel
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Widgets App")

        self.wind_label = QLabel()
        layout = QVBoxLayout()
        layout.addWidget(self.wind_label)

        widget = QWidget()
        widget.setLayout(layout)

        # Set the central widget of the Window. Widget will expand
        # to take up all the space in the window by default.
        self.setCentralWidget(widget)
        self.setWindowTitle("Sailbot Control")
    
    def show_boat_data(self, boat_data: BoatData):
        self.wind_label.setText("Wind: "+'{0:.2f}'.format(boat_data.wind.x)+", "+'{0:.2f}'.format(boat_data.wind.y)+", "+'{0:.2f}'.format(boat_data.wind.z))


def recieve_messages(listener: Listener):
    conn = listener.accept()
    while True:
        msg = conn.recv()
        time.sleep(1)
        # do something with msg


class RecieveMessages(QObject):
    listener: Listener
    progress = pyqtSignal(BoatData)

    def __init__(self, listener: Listener):
        super().__init__()
        self.listener = listener

    def run(self):
        conn = self.listener.accept()
        while True:
            msg = conn.recv()
            self.progress.emit(msg)
            # do something with msg


def CreateUI():
    app = QApplication(sys.argv)
    window = MainWindow()

    #Connect IPC
    address = ("localhost", 6000)
    listener = Listener(address, authkey=b"password")
    thread = QThread()
    worker = RecieveMessages(listener)
    worker.moveToThread(thread)
    worker.progress.connect(window.show_boat_data)
    thread.started.connect(worker.run)
    thread.start()

    window.show()
    app.exec()

    # exit
    listener.close()


class SailbotTest(Node):
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
        self.main_sail_publisher_ = self.create_publisher(
            Float64, "/main_sail_joint/cmd_pos", 10
        )
        self.rudder_publisher_ = self.create_publisher(
            Float64, "/rudder_joint/cmd_pos", 10
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.my_joystick = pygame.joystick.Joystick(0)
        self.my_joystick.init()
        self.clock = pygame.time.Clock()
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
        boat_data = BoatData()
        boat_data.wind = msg
        self.ui_conn.send(boat_data)

    def timer_callback(self):
        main_sail_msg = Float64()
        rudder_msg = Float64()
        for event in pygame.event.get():
            # print(str(self.my_joystick.get_axis(0))
            if event.type == 1536:
                # main sail
                main_sail_raw_value = self.my_joystick.get_axis(0)
                print(main_sail_raw_value)
                main_sail_radians = (
                    ((main_sail_raw_value + 1) * (math.pi - -math.pi)) / (2)
                ) - math.pi
                main_sail_msg.data = main_sail_radians
                self.main_sail_publisher_.publish(main_sail_msg)
                self.get_logger().info('Main sail: "%d"' % main_sail_msg.data)
                # rudder
                rudder_raw_value = self.my_joystick.get_axis(3)
                rudder_radians = (
                    ((rudder_raw_value + 1) * (math.pi - -math.pi)) / (2)
                ) - math.pi
                rudder_msg.data = rudder_radians
                self.rudder_publisher_.publish(rudder_msg)
                self.get_logger().info('rudder: "%d"' % rudder_msg.data)
            break


def main(args=None):
    rclpy.init(args=args)
    pygame.init()
    print("Joystics: " + str(pygame.joystick.get_count()))
    sailbot_test = SailbotTest()
    rclpy.spin(sailbot_test)
    sailbot_test.ui_proc.kill()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sailbot_test.destroy_node()
    rclpy.shutdown()


# do not use this
if __name__ == "__main__":
    main()
