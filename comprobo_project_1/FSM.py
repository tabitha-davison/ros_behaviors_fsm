import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class FSM(Node):
    def __init__(self):
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def state_handler(self): 
        # Handles transitions between states

    def star(self): 
        # Makes robot move in star

    def chase(self): 
        # Makes robot chase a person
        
    def lookout(self): 
        # Makes robot go through lookout sequence

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    rclpy.spin(node)
    rclpy.shutdown()
