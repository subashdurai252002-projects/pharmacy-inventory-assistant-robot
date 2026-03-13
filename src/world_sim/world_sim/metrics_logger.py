#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PoseArray
from std_msgs.msg import String


class MetricsLogger(Node):

    def __init__(self):
        super().__init__("metrics_logger")

        self.create_subscription(Pose2D, "/robot_pose", self.robot_cb, 10)
        self.create_subscription(PoseArray, "/humans", self.humans_cb, 10)
        self.create_subscription(String, "/task_status", self.status_cb, 10)

        self.robot_pose = None
        self.prev_pose = None
        self.humans = []

        self.start_time = None
        self.started = False

        self.path_length = 0.0
        self.min_human_distance = float("inf")

        self.near_collision_threshold = 0.6
        self.near_collision_count = 0

        self.prev_yaw = None
        self.smoothness_acc = 0.0
        self.smoothness_samples = 0

        self.get_logger().info("Metrics logger started")

    def robot_cb(self, msg):
        x = msg.x
        y = msg.y
        yaw = msg.theta

        self.robot_pose = (x, y, yaw)

        # only measure while experiment is active
        if not self.started:
            return

        if self.prev_pose is not None:
            dx = x - self.prev_pose[0]
            dy = y - self.prev_pose[1]
            step_dist = math.sqrt(dx * dx + dy * dy)
            self.path_length += step_dist

        self.prev_pose = (x, y)

        if self.prev_yaw is not None:
            dyaw = abs(yaw - self.prev_yaw)
            if dyaw > math.pi:
                dyaw = 2 * math.pi - dyaw

            self.smoothness_acc += dyaw
            self.smoothness_samples += 1

        self.prev_yaw = yaw

        for hx, hy in self.humans:
            dist = math.sqrt((x - hx) ** 2 + (y - hy) ** 2)

            if dist < self.min_human_distance:
                self.min_human_distance = dist

            if dist < self.near_collision_threshold:
                self.near_collision_count += 1

    def humans_cb(self, msg):
        self.humans = []

        for p in msg.poses:
            self.humans.append((p.position.x, p.position.y))

    def status_cb(self, msg):
        if msg.data == "STARTED":
            self.get_logger().info("Experiment started")

            self.started = True
            self.start_time = time.time()

            self.path_length = 0.0
            self.min_human_distance = float("inf")
            self.near_collision_count = 0
            self.smoothness_acc = 0.0
            self.smoothness_samples = 0

            if self.robot_pose is not None:
                self.prev_pose = (self.robot_pose[0], self.robot_pose[1])
                self.prev_yaw = self.robot_pose[2]
            else:
                self.prev_pose = None
                self.prev_yaw = None

        elif msg.data == "FINISHED":
            if not self.started or self.start_time is None:
                return

            elapsed = time.time() - self.start_time

            smoothness = 0.0
            if self.smoothness_samples > 0:
                smoothness = self.smoothness_acc / self.smoothness_samples

            self.get_logger().info("===== FINAL METRICS =====")
            self.get_logger().info(f"Task Time: {elapsed:.2f} s")
            self.get_logger().info(f"Path Length: {self.path_length:.2f} m")

            if self.min_human_distance == float("inf"):
                min_dist = "N/A"
            else:
                min_dist = f"{self.min_human_distance:.2f}"

            self.get_logger().info(f"Minimum Human Distance: {min_dist} m")
            self.get_logger().info(f"Near Collisions: {self.near_collision_count}")
            self.get_logger().info(f"Smoothness: {smoothness:.4f}")
            self.get_logger().info("=========================")

            self.started = False

    def print_metrics(self):
        # not used now; kept only if you want live metrics later
        pass


def main():
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
