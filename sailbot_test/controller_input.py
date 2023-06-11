import pygame
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64

class ControllerInput(Node):

    def __init__(self):
        super().__init__("controller_input")
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
    print("Joysticks: " + str(pygame.joystick.get_count()))
    controller_input = ControllerInput()
    rclpy.spin(controller_input)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_input.destroy_node()
    rclpy.shutdown()


# do not use this
if __name__ == "__main__":
    main()
