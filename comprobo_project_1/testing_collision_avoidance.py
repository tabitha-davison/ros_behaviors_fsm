import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from neato2_interfaces.msg import Bump 

# Subscribe to the sensor data via /scan and /bump
# Post process the sensor data to check for distances <0.3
# If threat detected, ovverird /cmd_vel and use stop function
# Subscribe to cmd_vel, subscribe to LIDAR


class CollisionAvoider(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        # Publish the velocity values
         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

         scan_subscription = self.create_subscription(LaserScan, "scan", 10)
         bump_subscription = self.create_subscription(Bump, "bump", 10)
         vel_subscription = self.create_subscription(Twist, "/cmd_vel", 10)

         self.near = False
         self.bumped = False
         self.latest_twist = Twist()

# Check if bumped
    def bump(self,msg):
        self.bumped = True

# Scan 
    def scan(self, msg)
        ranges = msg.ranges
        self.near = False
        min_dist = 0.3

        r = ranges
         if any(r < min_dist for r in front if r > 0.01):
            self.near= True

        self.publish_cmd()

    def publish_cmd(self)
    cmd = Twist()
        if self.near or self.bumped:
            print('Obstacle detected - stopping.')
            self.safe_cmd_pub.publish(cmd)  
        else:
            self.safe_cmd_pub.publish(self.latest_twist)


def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = CollisionAvoider()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

