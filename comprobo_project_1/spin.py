import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SpinNeato(Node):
    def __init__(self):
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def run_loop(self): 
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0.1

        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = SpinNeato()
    rclpy.spin(node)
    rclpy.shutdown()
