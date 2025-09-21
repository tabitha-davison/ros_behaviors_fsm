import rclpy
from rclpy.node import Node
from threading import Thread, Event
# from Time import sleep
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import Header
import math, time

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
        lidar_dist  = 10000 #DO THIS

        # transitions between FSM
        if lidar_dist < self.proximity:
            self.state = 1
        elif (self.state == 1 and lidar_dist > self.proximity):
            self.state = 2
            self.timepost = self.current_time
        elif (self.state == 2 and self.current_time >= self.timepost + (self.buffer * 3)):
            self.state = 0


    def star(self): 
   
        # Parameters
        forward_speed = 0.12                 # m/s
        turn_speed    = 0.35                 # rad/s
        edge_length   = 0.6                  # meters per star edge
        turn_angle    = math.radians(144.0)  # 144° turn for star
        heading_deg   = 36                   # rotate so one point faces up
        dt            = 0.02                 

        def drive(lin, ang, duration_s):
            """Publish velocity (lin, ang) for duration_s seconds, then stop."""
            msg = Twist()
            msg.linear.x  = lin
            msg.angular.z = ang
            end_time = time.time() + duration_s
            while time.time() < end_time:
                self.vel_pub.publish(msg)
                time.sleep(dt)
            # stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            time.sleep(0.1)

        # Initial orientation
        if abs(heading_deg) > 1e-6:
            # node.get_logger().info("Aligning heading")
            drive(0.0, turn_speed, math.radians(heading_deg) / turn_speed)

        # Draw the 5 edges
        for i in range(5):
            # node.get_logger().info(f"[Star] Edge {i+1}/5")
            drive(forward_speed, 0.0, edge_length / forward_speed)

            # node.get_logger().info(f"[Star] Turn {i+1}/5 (144°)")
            drive(0.0, turn_speed, turn_angle / turn_speed)

        # node.get_logger().info("Star complete.")

    def chase(self): 
        # Makes robot chase a person
        print(1)
        
    def lookout(self): 
        # Makes robot go through lookout sequence
        print(2)

    def time_loop(self):
        # Keeps track of time since the node begun
        time_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="neato_FSM")
        #print(time_header.stamp)
        self.time_pub.publish(time_header)
        
    def get_scan(self, msg):
        filler = 0

    def get_time(self, msg):
        # Gets the time of time thread
        sec_millis = msg.stamp.sec * 1000
        nano_millis = msg.stamp.nanosec / 1000
        self.current_time = sec_millis + nano_millis
        print(self.current_time)
        

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    rclpy.spin(node)
    rclpy.shutdown()
