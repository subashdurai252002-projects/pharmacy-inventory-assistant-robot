import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose2D
from world_sim.scenario_config import ROBOT_START

class WorldNode(Node):

    def __init__(self):
        super().__init__("world_node")

        # Publisher: robot pose
        self.pose_pub = self.create_publisher(Pose2D, "/robot_pose", 10)

        # Subscriber: velocity commands
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

        # Timer for simulation loop (20 Hz)
        self.timer = self.create_timer(0.05, self.update)

        # Robot state
        self.x = ROBOT_START[0]
        self.y = ROBOT_START[1]
        self.theta = ROBOT_START[2]

        # Velocity inputs
        self.v = 0.0
        self.w = 0.0

        self.get_logger().info("World node started")

    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        dt = 0.05

        # Update orientation
        self.theta += self.w * dt

        # Keep angle within [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi

        # Update position (differential drive)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        # Publish pose
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.theta

        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = WorldNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
