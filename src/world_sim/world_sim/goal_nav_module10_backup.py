import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from world_sim.scenario_config import OBJECTS, DROP_ZONE

def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GoalNav(Node):
    """
    Module 6:
    - waits for /orders (std_msgs/String) in format: "<name> <qty>"
      allowed names: paracetamol, vitamind, ibuprofen
    - drives robot by publishing /cmd_vel (geometry_msgs/Twist)
    - uses /robot_pose (geometry_msgs/Pose2D)
    """

    def __init__(self):
        super().__init__("goal_nav")

        # pubs/subs
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Pose2D, "/robot_pose", self.pose_cb, 10)
        self.create_subscription(String, "/orders", self.orders_cb, 10)
        self.create_subscription(PoseArray, "/human_predictions", self.predictions_cb, 10)
        self.timer = self.create_timer(0.05, self.step)  # 20 Hz

        # robot state
        self.pose: Pose2D | None = None

        # order state
        self.queue: list[tuple[str, int]] = []
        self.active_name: str | None = None
        self.active_qty: int = 0
        self.mode: str = "IDLE"  # GO_PICK, GO_DROP, IDLE
        self.human_predictions = []

        # ---- IMPORTANT: Put your real shelf coordinates here ----
        # We KNOW paracetamol shelf from your terminal:
        # "shelf id=1 at (-8.0, 15.0)"
        self.shelves = OBJECTS

        # drop location (your module 5 rotates at origin; drop often is origin)
        self.drop_xy = DROP_ZONE

        # control parameters
        self.stop_dist = 0.5
        self.k_lin = 0.8
        self.k_ang = 2.0
        self.max_v = 1.0
        self.max_w = 2.2
        self.turn_in_place_thresh = 0.7

        self.get_logger().info("Module 6 goal_nav ready. Waiting for orders.")

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
            self.get_logger().warn(f"Unknown item '{name}'. Allowed: {list(self.shelves.keys())}")
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

        # if idle, start immediately
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
        self.get_logger().info(
            f"Starting order: {self.active_qty} x {self.active_name} (at ({x}, {y}))"
        )

    def stop(self):
        self.cmd_pub.publish(Twist())

    def goto(self, tx: float, ty: float):
        # requires pose
        assert self.pose is not None

        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        err = wrap(desired - self.pose.theta)

        cmd = Twist()

        # safety check using predicted human positions
        safety_radius = 1.5
        for hx, hy in self.human_predictions:
            hdist = math.hypot(hx - self.pose.x, hy - self.pose.y)
            if hdist < safety_radius:
                self.stop()
                return dist

        # turn-in-place if pointing away a lot
        if abs(err) > self.turn_in_place_thresh:
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.max_w, min(self.max_w, self.k_ang * err))
        else:
            cmd.linear.x = max(0.0, min(self.max_v, self.k_lin * dist))
            cmd.angular.z = max(-self.max_w, min(self.max_w, self.k_ang * err))

        self.cmd_pub.publish(cmd)
        return dist

    def step(self):
        if self.pose is None:
            return

        # if idle, do nothing
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
                self.get_logger().info(f"Dropped 1 x {self.active_name}. Remaining: {self.active_qty}")

                if self.active_qty > 0:
                    self.mode = "GO_PICK"
                else:
                    self.get_logger().info("Order complete.")
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
