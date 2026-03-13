#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, PoseStamped

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def key_xy(x, y):
    # make a stable "ID" using rounded coordinates (since we don’t have object IDs)
    return (round(float(x), 2), round(float(y), 2))


class Viewer(Node):
    def __init__(self):
        super().__init__("viewer_node")

        # data
        self.robot_xy = (0.0, 0.0)
        self.robot_yaw = 0.0

        self.objects = []      # list of (x,y)
        self.detected = []     # list of (x,y)

        # NEW: reached/picked sets
        self.reached_set = set()   # keys (x,y)
        self.picked_set = set()

        # subs
        self.create_subscription(Odometry, "/odom", self.cb_odom, 10)
        self.create_subscription(PoseArray, "/objects", self.cb_objects, 10)
        self.create_subscription(PoseArray, "/detected", self.cb_detected, 10)

        # NEW: events
        self.create_subscription(PoseStamped, "/reached_object", self.cb_reached, 10)
        self.create_subscription(PoseStamped, "/picked_object", self.cb_picked, 10)

        self.get_logger().info("Viewer listening to: odom=/odom, objects=/objects, detected=/detected")
        self.get_logger().info("Also listening to: /reached_object (RED), /picked_object (hide)")

        # plot
        self.fig, self.ax = plt.subplots()
        self.ani = FuncAnimation(self.fig, self.update, interval=100)
        plt.title("Robot2D Viewer (Robot + Objects + Detected)")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.grid(True)

    def cb_odom(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robot_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def cb_objects(self, msg: PoseArray):
        self.objects = [(p.position.x, p.position.y) for p in msg.poses]

    def cb_detected(self, msg: PoseArray):
        self.detected = [(p.position.x, p.position.y) for p in msg.poses]

    def cb_reached(self, msg: PoseStamped):
        k = key_xy(msg.pose.position.x, msg.pose.position.y)
        # reached is before picked
        if k not in self.picked_set:
            self.reached_set.add(k)

    def cb_picked(self, msg: PoseStamped):
        k = key_xy(msg.pose.position.x, msg.pose.position.y)
        self.picked_set.add(k)
        # once picked, no need to keep "reached"
        if k in self.reached_set:
            self.reached_set.remove(k)

    def update(self, frame):
        self.ax.clear()
        self.ax.grid(True)
        self.ax.set_title("Robot2D Viewer (Robot + Objects + Detected)")
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")

        # robot
        rx, ry = self.robot_xy
        self.ax.scatter([rx], [ry], label="robot")

        # heading line
        hx = rx + 0.8 * math.cos(self.robot_yaw)
        hy = ry + 0.8 * math.sin(self.robot_yaw)
        self.ax.plot([rx, hx], [ry, hy], label="heading")

        # build lists
        obj_xy = [(float(x), float(y)) for (x, y) in self.objects]
        det_xy = [(float(x), float(y)) for (x, y) in self.detected]

        # 1) BLUE objects (but NOT picked)
        blue = []
        for (x, y) in obj_xy:
            if key_xy(x, y) not in self.picked_set:
                blue.append((x, y))

        if blue:
            bx, by = zip(*blue)
            self.ax.scatter(bx, by, label="objects")

        # 2) ORANGE detected (but NOT picked, and NOT reached-red)
        orange = []
        for (x, y) in det_xy:
            k = key_xy(x, y)
            if k in self.picked_set:
                continue
            if k in self.reached_set:
                continue
            orange.append((x, y))

        if orange:
            ox, oy = zip(*orange)
            self.ax.scatter(ox, oy, label="detected")

        # 3) RED reached (but NOT picked)
        red = []
        for k in self.reached_set:
            if k not in self.picked_set:
                red.append(k)

        if red:
            rx2, ry2 = zip(*red)
            # matplotlib default colors: next color in cycle, but label will show as reached
            self.ax.scatter(rx2, ry2, label="reached")

        # axes limits (auto but stable)
        self.ax.set_xlim(-10, 25)
        self.ax.set_ylim(-5, 16)

        self.ax.legend(loc="upper right")


def main():
    rclpy.init()
    node = Viewer()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
