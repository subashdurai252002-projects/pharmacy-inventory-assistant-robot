#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from world_sim.scenario_config import OBJECTS, DROP_ZONE


class HumanSim(Node):
    def __init__(self):
        super().__init__("human_sim")

        self.pub_humans = self.create_publisher(PoseArray, "/humans", 10)

        # Important warehouse points
        self.base_points = list(OBJECTS.values()) + [
            DROP_ZONE,
            (10.0, 5.0),   # center crossing point
            (15.0, 0.0),   # another crossing point
        ]

        # Create 5 humans at different starting places
        start_points = self.base_points[:5]

        self.humans = []
        speeds = [0.9, 1.0, 1.1, 0.95, 1.05]

        for i in range(5):
            sx, sy = start_points[i]
            tx, ty = self.get_random_target(exclude=(sx, sy))

            self.humans.append({
                "x": float(sx),
                "y": float(sy),
                "speed": speeds[i],
                "target": (tx, ty)
            })

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info("human_sim started (5 humans with random wandering)")

    def get_random_target(self, exclude=None):
        candidates = []
        for p in self.base_points:
            if exclude is None or p != exclude:
                candidates.append(p)

        tx, ty = random.choice(candidates)

        # small random offset so humans don't go to exact same point
        tx += random.uniform(-1.5, 1.5)
        ty += random.uniform(-1.5, 1.5)

        return tx, ty

    def step(self):
        msg = PoseArray()

        for h in self.humans:
            tx, ty = h["target"]

            dx = tx - h["x"]
            dy = ty - h["y"]
            dist = math.hypot(dx, dy)

            # If reached current target, choose a new random one
            if dist < 0.4:
                h["target"] = self.get_random_target(exclude=(tx, ty))
                tx, ty = h["target"]
                dx = tx - h["x"]
                dy = ty - h["y"]
                dist = math.hypot(dx, dy)

            if dist > 1e-6:
                ux = dx / dist
                uy = dy / dist
                h["x"] += ux * h["speed"] * self.dt
                h["y"] += uy * h["speed"] * self.dt

            p = Pose()
            p.position.x = h["x"]
            p.position.y = h["y"]
            p.position.z = 0.0
            p.orientation.w = 1.0
            msg.poses.append(p)

        self.pub_humans.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanSim()
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
