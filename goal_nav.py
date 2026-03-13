#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray, PoseStamped


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class GoalNavNode(Node):
    def __init__(self):
        super().__init__('goal_nav_node')

        # State
        self.state = "SEARCH"
        self.robot_xy = (0.0, 0.0)
        self.robot_yaw = 0.0

        self.detected = []   # list of (x,y)
        self.target = None   # (x,y)

        # Params
        self.reach_dist = 0.6
        self.drop_xy = (0.0, 0.0)

        # ROS I/O
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_det = self.create_subscription(PoseArray, '/detected', self.detected_cb, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # NEW topics for viewer
        self.pub_reached = self.create_publisher(PoseStamped, '/reached_object', 10)
        self.pub_picked = self.create_publisher(PoseStamped, '/picked_object', 10)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("✅ goal_nav_node started (SEARCH -> GO_TO_OBJECT -> PICK -> GO_TO_DROP -> DROP -> DONE)")

    def odom_cb(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def detected_cb(self, msg: PoseArray):
        self.detected = [(p.position.x, p.position.y) for p in msg.poses]

    def publish_reached(self, xy):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x = float(xy[0])
        m.pose.position.y = float(xy[1])
        m.pose.orientation.w = 1.0
        self.pub_reached.publish(m)

    def publish_picked(self, xy):
        m = PoseStamped()
        m.header.frame_id = "map"
        m.pose.position.x = float(xy[0])
        m.pose.position.y = float(xy[1])
        m.pose.orientation.w = 1.0
        self.pub_picked.publish(m)

    def stop(self):
        self.pub_cmd.publish(Twist())

    def loop(self):
        cmd = Twist()

        # SEARCH: rotate until something appears in /detected
        if self.state == "SEARCH":
            if len(self.detected) == 0:
                cmd.angular.z = 0.6
                cmd.linear.x = 0.0
                self.pub_cmd.publish(cmd)
                return
            else:
                self.target = self.detected[0]
                self.get_logger().info(f"🔎 Detected target -> GO_TO_OBJECT {self.target}")
                self.state = "GO_TO_OBJECT"

        # GO_TO_OBJECT: drive to target
        if self.state == "GO_TO_OBJECT" and self.target is not None:
            rx, ry = self.robot_xy
            tx, ty = self.target

            dx = tx - rx
            dy = ty - ry
            dist = math.hypot(dx, dy)

            desired_yaw = math.atan2(dy, dx)
            yaw_err = desired_yaw - self.robot_yaw
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            # simple controller
            cmd.angular.z = clamp(1.5 * yaw_err, -1.0, 1.0)

            # move forward only if roughly facing target
            if abs(yaw_err) < 0.6:
                cmd.linear.x = clamp(0.8 * dist, 0.0, 0.8)
            else:
                cmd.linear.x = 0.0

            self.pub_cmd.publish(cmd)

            # reached?
            if dist < self.reach_dist:
                self.stop()
                self.publish_reached(self.target)
                self.get_logger().info("✅ Reached object -> PICK")
                self.state = "PICK"
            return

        # PICK: publish picked event, then go to drop
        if self.state == "PICK" and self.target is not None:
            self.publish_picked(self.target)
            self.get_logger().info("📦 PICK done -> GO_TO_DROP")
            self.state = "GO_TO_DROP"
            return

        # GO_TO_DROP
        if self.state == "GO_TO_DROP":
            rx, ry = self.robot_xy
            tx, ty = self.drop_xy

            dx = tx - rx
            dy = ty - ry
            dist = math.hypot(dx, dy)

            desired_yaw = math.atan2(dy, dx)
            yaw_err = desired_yaw - self.robot_yaw
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

            cmd.angular.z = clamp(1.5 * yaw_err, -1.0, 1.0)
            if abs(yaw_err) < 0.6:
                cmd.linear.x = clamp(0.8 * dist, 0.0, 0.8)
            else:
                cmd.linear.x = 0.0

            self.pub_cmd.publish(cmd)

            if dist < 0.7:
                self.stop()
                self.get_logger().info("✅ Reached drop -> DROP")
                self.state = "DROP"
            return

        # DROP
        if self.state == "DROP":
            self.get_logger().info("✅ DROP done -> DONE")
            self.state = "DONE"
            return

        # DONE
        if self.state == "DONE":
            self.stop()
            return


def main():
    rclpy.init()
    node = GoalNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
