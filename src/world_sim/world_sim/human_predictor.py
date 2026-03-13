#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose


class HumanPredictor(Node):
    def __init__(self):
        super().__init__("human_predictor")

        self.sub_humans = self.create_subscription(
            PoseArray,
            "/humans",
            self.humans_cb,
            10
        )

        self.pub_predictions = self.create_publisher(
            PoseArray,
            "/human_predictions",
            10
        )

        self.prev_positions = None
        self.curr_positions = None

        self.dt = 0.1              # same as human_sim timer
        self.horizon_steps = 10    # predict 10 future steps

        self.get_logger().info("human_predictor started")

    def humans_cb(self, msg: PoseArray):
        # store previous positions
        if self.curr_positions is not None:
            self.prev_positions = self.curr_positions.copy()

        # store current positions
        self.curr_positions = []
        for p in msg.poses:
            self.curr_positions.append((p.position.x, p.position.y))

        # if we have both previous and current positions, predict
        if self.prev_positions is not None:
            self.publish_predictions()

    def publish_predictions(self):
        msg = PoseArray()

        n = min(len(self.prev_positions), len(self.curr_positions))

        for i in range(n):
            x_prev, y_prev = self.prev_positions[i]
            x_curr, y_curr = self.curr_positions[i]

            # estimate velocity
            vx = (x_curr - x_prev) / self.dt
            vy = (y_curr - y_prev) / self.dt

            # predict future positions
            for k in range(1, self.horizon_steps + 1):
                p = Pose()
                p.position.x = x_curr + vx * self.dt * k
                p.position.y = y_curr + vy * self.dt * k
                p.position.z = 0.0

                # store human id in orientation.w
                p.orientation.w = float(i)

                msg.poses.append(p)

        self.pub_predictions.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanPredictor()
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
