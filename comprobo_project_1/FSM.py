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
        self.state = 0 # the state handler variable. 0: star, 1: chase, 2: lookout
        self.timepost = 0 # the time lookout() state change in ms
        self.buffer = 500 # the time of a lookout() swing in ms
        self.proximity = 1 # the distance of detecting to chase in m
        self.current_time = 0.0;
        self.closest_dist = 0;
        self.closest_dist_rad = 0.0;

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
        
        print(f"CLOSEST DIST IS: {self.closest_dist}")
        print(f"CLOSEST DIST ANG IS: {self.closest_dist_rad}")
        print(f"state var is: {self.state}")

        # run the current state
        match self.state:
            case 0:
                #self.star()
                print(f"in state 0")
            case 1:
                #self.chase()
                print(f"in state 1")
            case 2:
                #self.lookout()
                print(f"in state 2")
        
    def state_handler(self): 
        """Handles transitions between states"""
        # find the closest lidar distance
        lidar_dist = self.closest_dist

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
            while time.time() < end_time and self.state == 0:
                self.vel_pub.publish(msg)
                time.sleep(dt)
            # stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            time.sleep(0.1)

        # Initial orientation
        if abs(heading_deg) > 1e-6:
            drive(0.0, turn_speed, math.radians(heading_deg) / turn_speed)

        # Draw the 5 edges
        for i in range(5):
            if (self.state ==0):
                drive(forward_speed, 0.0, edge_length / forward_speed)
                drive(0.0, turn_speed, turn_angle / turn_speed)

    def chase(self): 
        # Makes robot chase a person
        print(f"chasing robot (state 1)")
        
    def lookout(self): 

        # Parameters
        turn_speed = 0.35                 # rad/s
        turn_angle = math.radians(120.0)  # 120° in radians
        dt         = 0.02                 # 

        def turn(ang_vel, duration_s):
            """Publish angular velocity for duration_s seconds, then stop."""
            msg = Twist()
            msg.angular.z = ang_vel
            end_time = time.time() + duration_s
            while time.time() < end_time and self.state == 2:
                self.vel_pub.publish(msg)
                time.sleep(dt)
            # stop
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            time.sleep(0.1)

        # Pan sequence
        if (self.state == 2):
            # Left 120°
            turn(turn_speed, turn_angle / turn_speed)

        if (self.state == 2):
            # Right 120° (negative angular velocity)
            turn(-turn_speed, turn_angle / turn_speed)

    def time_loop(self):
        # Keeps track of time since the node begun
        time_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="neato_FSM")
        self.time_pub.publish(time_header)
        
    def get_scan(self, msg):
        # len(msg.ranges) = 361
        lower_post = .25 #distance in meters to filter below
        upper_post = self.proximity #distance in meters to filter above
        filtered_dist_av = 0;
        filtered_ang_av = 0;
        average_dividend = 0;
        for i in range(len(msg.ranges) - 1):
            if lower_post < msg.ranges[i] and msg.ranges[i] < upper_post:
                filtered_dist_av = filtered_dist_av + msg.ranges[i]
                filtered_ang_av = filtered_ang_av + i 
                average_dividend = average_dividend + 1
        
        if(average_dividend is not 0):
            self.closest_dist = filtered_dist_av / average_dividend
            self.closest_dist_rad = filtered_ang_av / average_dividend
        else:
            self.closest_dist = self.proximity + 1
            self.closest_dist_rad = 0


    def get_time(self, msg):
        # Gets the time of time thread
        sec_millis = msg.stamp.sec * 1000
        nano_millis = msg.stamp.nanosec / 1000
        self.current_time = sec_millis + nano_millis
        

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    rclpy.spin(node)
    rclpy.shutdown()
