#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose2D, PoseArray
from std_msgs.msg import String
from world_sim.scenario_config import OBJECTS, DROP_ZONE


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GoalNav(Node):
    """
    Module 6 + Module 10A/10B practical version

    - waits for /orders (std_msgs/String) in format: "<name> <qty>"
    - uses /robot_pose (geometry_msgs/Pose2D)
    - uses /human_predictions (geometry_msgs/PoseArray)
    - publishes /cmd_vel (geometry_msgs/Twist)

    Order flow:
      GO_PICK -> GO_DROP -> repeat until qty done

    Prediction-aware behavior:
      - ignores humans behind the robot
      - very close human ahead -> back off + turn away
      - close human ahead -> slow down strongly + turn away
      - moderate human ahead -> detour
    """

    def __init__(self):
        super().__init__("goal_nav")

        # pubs/subs
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/task_status", 10)
        self.create_subscription(Pose2D, "/robot_pose", self.pose_cb, 10)
        self.create_subscription(String, "/orders", self.orders_cb, 10)
        self.create_subscription(PoseArray, "/human_predictions", self.predictions_cb, 10)

        self.timer = self.create_timer(0.05, self.step)  # 20 Hz

        # robot state
        self.pose: Pose2D | None = None

        # predicted human positions
        self.human_predictions: list[tuple[float, float]] = []

        # order state
        self.queue: list[tuple[str, int]] = []
        self.active_name: str | None = None
        self.active_qty: int = 0
        self.mode: str = "IDLE"  # GO_PICK, GO_DROP, IDLE

        # shelf coordinates
        self.shelves = {
            "paracetamol": (-8.0, 15.0),
            "vitamin_c": (16.0, 15.0),
            "ibuprofen": (25.0, -5.0),
            "aspirin": (-5.0, -5.0),
        }

        # drop location
        self.drop_xy = (0.0, 0.0)

        # control parameters
        self.stop_dist = 0.5
        self.k_lin = 0.8
        self.k_ang = 2.0
        self.max_v = 1.0
        self.max_w = 2.2
        self.turn_in_place_thresh = 0.7

        self.get_logger().info("Module 10 goal_nav ready. Waiting for orders.")

    def pose_cb(self, msg: Pose2D):
        self.pose = msg

    def orders_cb(self, msg: String):
        raw = msg.data.strip().lower()
        parts = raw.split()

        if len(parts) != 2:
            self.get_logger().warn(f"Bad order format: '{msg.data}' (use: Name Quantity)")
            return

        name, qty_s = parts[0], parts[1]

        if name not in self.shelves:
            self.get_logger().warn(
                f"Unknown item '{name}'. Allowed: {list(self.shelves.keys())}"
            )
            return

        try:
            qty = int(qty_s)
        except ValueError:
            self.get_logger().warn(f"Bad quantity: '{qty_s}'")
            return

        if qty <= 0:
            self.get_logger().warn("Quantity must be > 0")
            return

        self.queue.append((name, qty))
        self.get_logger().info(f"Added {qty} x {name}")

        if self.mode == "IDLE" and self.active_name is None:
            self.start_next()

    def predictions_cb(self, msg: PoseArray):
        self.human_predictions = []
        for p in msg.poses:
            self.human_predictions.append((p.position.x, p.position.y))

    def start_next(self):
        if not self.queue:
            self.active_name = None
            self.active_qty = 0
            self.mode = "IDLE"
            self.stop()
            self.get_logger().info("No more orders. Waiting for orders.")
            return

        self.active_name, self.active_qty = self.queue.pop(0)
        x, y = self.shelves[self.active_name]
        self.mode = "GO_PICK"

        msg = String()
        msg.data = "STARTED"
        self.status_pub.publish(msg)
        self.get_logger().info(
            f"Starting order: {self.active_qty} x {self.active_name} (at ({x}, {y}))"
        )

    def stop(self):
        self.cmd_pub.publish(Twist())

    def goto(self, tx: float, ty: float):
        assert self.pose is not None

        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        err = wrap(desired - self.pose.theta)

        cmd = Twist()

        # -------------------------------
        # Prediction-aware detour logic
        # -------------------------------
        detour_radius = 1.5
        slow_radius = 1.0
        stop_radius = 0.55
        forward_dot_threshold = 0.0

        heading_x = math.cos(self.pose.theta)
        heading_y = math.sin(self.pose.theta)

        avoid_turn = 0.0
        speed_scale = 1.0

        for hx, hy in self.human_predictions:
            rel_x = hx - self.pose.x
            rel_y = hy - self.pose.y
            hdist = math.hypot(rel_x, rel_y)

            # only care about humans in front of robot
            dot = rel_x * heading_x + rel_y * heading_y
            if dot <= forward_dot_threshold:
                continue

            if hdist < detour_radius:
                # side > 0 => human on left, side < 0 => human on right
                side = heading_x * rel_y - heading_y * rel_x

                # strongest reaction: too close -> back off and turn away
                if hdist < stop_radius:
                    cmd.linear.x = -0.2
                    cmd.angular.z = -1.4 if side > 0 else 1.4
                    self.cmd_pub.publish(cmd)
                    return dist

                # medium reaction: slow strongly and turn away
                if hdist < slow_radius:
                    speed_scale = min(speed_scale, 0.2)
                    avoid_turn += -1.4 if side > 0 else 1.4

                # weaker reaction: detour and reduce speed
                else:
                    speed_scale = min(speed_scale, 0.5)
                    avoid_turn += -0.8 if side > 0 else 0.8

        # combine goal heading + avoidance turn
        total_ang = self.k_ang * err + avoid_turn

        # normal desired forward speed
        base_v = max(0.0, min(self.max_v, self.k_lin * dist))

        if abs(avoid_turn) > 0.01:
            cmd.linear.x = max(0.12, base_v * speed_scale)
            cmd.angular.z = max(-self.max_w, min(self.max_w, total_ang))
        else:
            if abs(err) > self.turn_in_place_thresh:
                cmd.linear.x = 0.0
                cmd.angular.z = max(-self.max_w, min(self.max_w, total_ang))
            else:
                cmd.linear.x = base_v
                cmd.angular.z = max(-self.max_w, min(self.max_w, total_ang))

        self.cmd_pub.publish(cmd)
        return dist

    def step(self):
        if self.pose is None:
            return

        if self.mode == "IDLE":
            return

        if self.active_name is None:
            self.start_next()
            return

        if self.mode == "GO_PICK":
            tx, ty = self.shelves[self.active_name]
            dist = self.goto(tx, ty)
            if dist < self.stop_dist:
                self.stop()
                self.get_logger().info(f"Picked 1 x {self.active_name}")
                self.mode = "GO_DROP"

        elif self.mode == "GO_DROP":
            tx, ty = self.drop_xy
            dist = self.goto(tx, ty)
            if dist < self.stop_dist:
                self.stop()
                self.active_qty -= 1
                self.get_logger().info(
                    f"Dropped 1 x {self.active_name}. Remaining: {self.active_qty}"
                )

                if self.active_qty > 0:
                    self.mode = "GO_PICK"
                else:
                    self.get_logger().info("Order complete.")

                    msg = String()
                    msg.data = "FINISHED"
                    self.status_pub.publish(msg)

                    self.active_name = None
                    self.start_next()


def main():
    rclpy.init()
    node = GoalNav()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
