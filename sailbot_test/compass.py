import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class Compass(Node):

    def __init__(self):
        super().__init__("compass")
        self.anemometer_subscriber = self.create_subscription(
            TFMessage,
            "/world/waves/dynamic_pose/info",
            self.joint_state_callback,
            0,
        )

    def joint_state_callback(self, msg: TFMessage):
        transform: TransformStamped
        for transform in msg.transforms:
            print(transform.child_frame_id)
            #if transform.child_frame_id == "rs750":
            #    print(transform.transform)
        pass


def main(args=None):
    rclpy.init(args=args)
    compass = Compass()
    rclpy.spin(compass)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    compass.destroy_node()
    rclpy.shutdown()


# do not use this
if __name__ == "__main__":
    main()
