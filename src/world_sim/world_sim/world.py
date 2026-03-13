#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


def pose_xy(x, y):
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = 0.0
    p.orientation.w = 1.0
    return p


class WorldSim(Node):
    def __init__(self):
        super().__init__("world_sim")

        # Robot initial pose
        self.x = 0.0
        self.y = 20.0
        self.yaw = 0.0

        # Commanded velocity
        self.v = 0.0
        self.w = 0.0

        # Sensor range for "mock detection"
        self.SENSOR_RANGE = 12.0

        # Fixed objects (Option A fixed positions)
        self.objects = [
            (8.0, 2.0),
            (16.0, 2.0),
            (24.0, 0.0),
        ]

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_objects = self.create_publisher(PoseArray, "/objects", 10)
        self.pub_detected = self.create_publisher(PoseArray, "/detected", 10)

        # Subscribers
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.sub_picked = self.create_subscription(PoseArray, "/picked", self.on_picked, 10)

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info("✅ world_node started")

    def on_cmd(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def on_picked(self, msg: PoseArray):
        # Remove picked objects from the world
        for pp in msg.poses:
            px = float(pp.position.x)
            py = float(pp.position.y)

            new_list = []
            removed = False
            for (ox, oy) in self.objects:
                if (not removed) and math.hypot(ox - px, oy - py) < 0.6:
                    removed = True
                    continue
                new_list.append((ox, oy))
            self.objects = new_list

    def step(self):
        # Integrate robot motion
        self.yaw += self.w * self.dt
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt

        # Publish odom
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.orientation.z = float(math.sin(self.yaw / 2.0))
        odom.pose.pose.orientation.w = float(math.cos(self.yaw / 2.0))
        self.pub_odom.publish(odom)

        # Publish objects (remaining)
        obj = PoseArray()
        obj.header = Header()
        obj.header.frame_id = "map"
        obj.poses = [pose_xy(x, y) for (x, y) in self.objects]
        self.pub_objects.publish(obj)

        # Publish detected objects (within sensor range)
        det = PoseArray()
        det.header = Header()
        det.header.frame_id = "map"
        det_list = []
        for (ox, oy) in self.objects:
            if math.hypot(ox - self.x, oy - self.y) <= self.SENSOR_RANGE:
                det_list.append(pose_xy(ox, oy))
        det.poses = det_list
        self.pub_detected.publish(det)


def main():
    rclpy.init()
    node = WorldSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
