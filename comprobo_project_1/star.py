"""
Draw Star
---------

"""

import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep, time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math


class DrawStar(Node):


    def __init__(self):
        super().__init__('draw_star_with_estop')

        # Parameters 
        self.forward_speed = 0.12             # m/s
        self.turn_speed = 0.35                # rad/s (positive = left)
        self.edge_length = 0.6                # meters per star edge
        self.turn_angle = math.radians(144.0) # 144° left turn makes the star egdes
        self.initial_heading_offset_deg = 36  # rotate so top point faces up
        self.rate_dt = 0.02                   # seconds, estop check frequency
        # 

        self.e_stop = Event()
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)

        # Background thread for the star running loop
        self.run_loop_thread = Thread(target=self.run_loop, daemon=True)
        self.run_loop_thread.start()

    # Motion primitives 

    def handle_estop(self, msg: Bool):
        """Handle estop messages (True = stop)"""
        if msg.data:
            self.e_stop.set()
            self.drive(0.0, 0.0)

    def drive(self, linear: float, angular: float):
        """Publish specified linear (x) and angular (z) velocity."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def drive_forward(self, distance: float):
        """Drive straight """
        if distance <= 0:
            return
        duration = distance / max(self.forward_speed, 1e-6)
        end_time = time() + duration

        if not self.e_stop.is_set():
            self.drive(self.forward_speed, 0.0)

        while time() < end_time:
            if self.e_stop.is_set():
                break
            sleep(self.rate_dt)

        self.drive(0.0, 0.0)

    def turn_left(self, angle_rad: float):
        """Turn left """
        if angle_rad <= 0:
            return
        duration = angle_rad / max(self.turn_speed, 1e-6)
        end_time = time() + duration

        if not self.e_stop.is_set():
            self.drive(0.0, self.turn_speed)

        while time() < end_time:
            if self.e_stop.is_set():
                break
            sleep(self.rate_dt)

        self.drive(0.0, 0.0)

    # Star Pattern runner 

    def run_loop(self):
        """Executes the star path: 5 edges with 144° turns between."""
        self.drive(0.0, 0.0)
        sleep(0.5)

        # Initial orientation so the star has one point facing up
        if not self.e_stop.is_set() and abs(self.initial_heading_offset_deg) > 1e-6:
            self.get_logger().info("Aligning heading for star orientation")
            self.turn_left(math.radians(self.initial_heading_offset_deg))

        for i in range(5):
            if self.e_stop.is_set():
                break
            self.get_logger().info(f"[Star] Edge {i+1}/5")
            self.drive_forward(self.edge_length)

            if self.e_stop.is_set():
                break
            self.get_logger().info(f"[Star] Turn {i+1}/5 (144°)")
            self.turn_left(self.turn_angle)

        self.get_logger().info("Done with star run (or estopped).")


def main(args=None):
    rclpy.init(args=args)
    node = DrawStar()
    try:
        rclpy.spin(node)
    finally:
        node.drive(0.0, 0.0)  # ensure stop on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
