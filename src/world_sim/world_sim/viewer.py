#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PoseArray
from world_sim.scenario_config import OBJECTS, DROP_ZONE

# Fixed shelf positions from scenario config
ITEM_LOCATIONS = OBJECTS.copy()

WORLD_XLIM = (-10, 30)
WORLD_YLIM = (-10, 20)


class Viewer(Node):
    def __init__(self):
        super().__init__("viewer")

        self.pose = None
        self.held_item = None
        self.carried_item_pos = None
        self.dropped_items = {}   # {"paracetamol": 2, "vitamin_c": 1}
        self.humans = []

        # Subscribe robot pose
        self.sub = self.create_subscription(
            Pose2D, "/robot_pose", self.pose_cb, 10
        )

        # Subscribe humans
        self.create_subscription(
            PoseArray,
            "/humans",
            self.humans_cb,
            10
        )

        # Terminal-only mode
        self.terminal_mode = ("--terminal" in sys.argv)

        if self.terminal_mode:
            self.get_logger().info("Viewer started (terminal mode).")
            self.get_logger().info("Waiting for /robot_pose ...")
            return

        # GUI mode
        self.get_logger().info("Viewer started (GUI mode).")
        import matplotlib.pyplot as plt
        plt.ion()

        self.plt = plt
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title("Robot Viewer")

        # Update loop
        self.timer = self.create_timer(0.1, self.update_plot)

        # Close handling
        self.fig.canvas.mpl_connect("close_event", self.on_close)

        self.plt.ion()
        self.plt.show(block=False)

    def pose_cb(self, msg: Pose2D):
        self.pose = msg

        # carried object moves with robot
        if self.held_item is not None:
            self.carried_item_pos = (msg.x, msg.y)

        if self.terminal_mode:
            self.get_logger().info(
                f"POSE: ({msg.x:.2f},{msg.y:.2f}) th={msg.theta:.2f}"
            )

    def humans_cb(self, msg):
        self.humans = []
        for p in msg.poses:
            self.humans.append((p.position.x, p.position.y))

    def update_pick_drop_state(self):
        if self.pose is None:
            return

        drop_x, drop_y = DROP_ZONE

        # pick logic
        if self.held_item is None:
            for name, pos in ITEM_LOCATIONS.items():
                dx = self.pose.x - pos[0]
                dy = self.pose.y - pos[1]
                dist = (dx**2 + dy**2) ** 0.5

                if dist < 0.5:
                    self.held_item = name
                    self.carried_item_pos = (self.pose.x, self.pose.y)
                    break

        # drop logic
        if self.held_item is not None:
            dx = self.pose.x - drop_x
            dy = self.pose.y - drop_y
            dist_drop = (dx**2 + dy**2) ** 0.5

            if dist_drop < 0.5:
                if self.held_item in self.dropped_items:
                    self.dropped_items[self.held_item] += 1
                else:
                    self.dropped_items[self.held_item] = 1

                self.held_item = None
                self.carried_item_pos = None

    def update_plot(self):
        if self.terminal_mode:
            return
        if self.pose is None:
            return

        self.update_pick_drop_state()

        x = float(self.pose.x)
        y = float(self.pose.y)
        th = float(self.pose.theta)

        self.ax.clear()
        self.ax.set_title("Robot Viewer")
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_xlim(*WORLD_XLIM)
        self.ax.set_ylim(*WORLD_YLIM)
        self.ax.grid(True)

        # draw fixed shelf objects
        shelf_x = [p[0] for p in ITEM_LOCATIONS.values()]
        shelf_y = [p[1] for p in ITEM_LOCATIONS.values()]
        self.ax.scatter(shelf_x, shelf_y, s=80, color="blue")

        for name, (ix, iy) in ITEM_LOCATIONS.items():
            self.ax.text(ix + 0.4, iy + 0.4, name, fontsize=10, color="black")

        # draw drop point
        self.ax.scatter([DROP_ZONE[0]], [DROP_ZONE[1]], marker="X", s=120, color="orange")
        self.ax.text(DROP_ZONE[0] + 0.4, DROP_ZONE[1] + 0.4, "DROP", fontsize=10, color="black")

        # draw dropped item counts near drop zone
        for i, (name, count) in enumerate(self.dropped_items.items()):
            dx = 0.8 * (i % 3)
            dy = 0.8 * (i // 3)
            px = DROP_ZONE[0] + dx
            py = DROP_ZONE[1] + dy
            self.ax.scatter([px], [py], s=70, color="orange")
            self.ax.text(px + 0.15, py + 0.15, f"{name} x {count}", fontsize=9, color="orange")

        # draw humans
        if len(self.humans) > 0:
            xs = [h[0] for h in self.humans]
            ys = [h[1] for h in self.humans]
            self.ax.scatter(xs, ys, s=80, color="red")
            for i, (hx, hy) in enumerate(self.humans):
                self.ax.text(hx + 0.2, hy + 0.2, f"H{i+1}", fontsize=8, color="red")

        # draw carried item slightly in front of robot
        if self.held_item is not None and self.carried_item_pos is not None:
            offset = 0.8
            cx = x + offset * math.cos(th)
            cy = y + offset * math.sin(th)
            self.ax.scatter([cx], [cy], s=120, color="red")
            self.ax.text(cx + 0.2, cy + 0.2, f"{self.held_item}", fontsize=9, color="red")

        # draw robot
        self.ax.scatter([x], [y], s=80, color="green")
        dx = 1.0 * math.cos(th)
        dy = 1.0 * math.sin(th)
        self.ax.arrow(
            x, y, dx, dy,
            head_width=0.4,
            head_length=0.5,
            length_includes_head=True,
            color="green"
        )

        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def on_close(self, event):
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
