import rclpy
from rclpy.node import Node
from threading import Thread, Event
# from Time import sleep
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import Header
import math

class FSM(Node):
    """A signel node ros2 program that transforms a neato into the
    ultimate law keeping device: aka. THE SHERIFF"""

    def __init__(self):
        # Name the node discoverable
        super().__init__('sheriff_fsm')

        # global variables
        self.state = 0 # the state handler variable
        self.timepost = 0 # the time lookout() state change in ms
        self.buffer = 500 # the time of a lookout() swing in ms
        self.proximity = 1 # the distance of detecting to chase in m
        self.current_time = 0.0;
        self.closest_dist = 0;
        self.closest_dist_ang = 0;

        # pubs & subs
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.time_pub = self.create_publisher(Header, 'FSM_time', 10) # maybe unnecessary
        self.create_subscription(LaserScan, 'scan', self.get_scan, 10)
        self.create_subscription(Header, 'FSM_time', self.get_time, 1)
        
        # Run threads
        self.create_timer(0.1, self.run_threads)

    def run_threads(self):
        self.run_loop_thread = Thread(target=self.run_loop())
        self.run_time_loop_thread = Thread(target=self.time_loop())
        self.run_time_loop_thread.start()
        self.run_loop_thread.start()

    def run_loop(self):
        """run the robot states"""
        # check transitions
        self.state_handler()

        # run the current state
        match self.state:
            case 0:
                self.star()
            case 1:
                self.chase()
            case 2:
                self.lookout()
        
    def state_handler(self): 
        """Handles transitions between states"""
        # find the closest lidar distance
        lidar_dist  = 0 #DO THIS

        # transitions between FSM
        if lidar_dist < self.proximity:
            self.state = 1
        elif (self.state == 1 and lidar_dist < self.proximity):
            self.state = 2
            self.timepost = self.current_time
            print(f"timepost is: {self.timepost}")
        elif (self.state == 2 and self.current_time >= self.timepost + (self.buffer * 3)):
            self.state = 0


    def star(self): 
        # Makes robot move in star
        print(f"running routes (state 0)")

    def chase(self): 
        # Makes robot chase a person
        print(f"chasing robot (state 1)")
        
    def lookout(self): 
        # Makes robot go through lookout sequence
        print(f"we've lost 'em (state 2)")

    def time_loop(self):
        # Keeps track of time since the node begun
        time_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="neato_FSM")
        #print(time_header.stamp)
        self.time_pub.publish(time_header)
        
    def get_scan(self, msg):
        print(msg)

    def get_time(self, msg):
        # Gets the time of time thread
        sec_millis = msg.stamp.sec * 1000
        nano_millis = msg.stamp.nanosec / 1000
        self.current_time = sec_millis + nano_millis
        #print(self.current_time)
        

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    rclpy.spin(node)
    rclpy.shutdown()
