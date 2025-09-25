import rclpy
from rclpy.node import Node
from threading import Thread, Event
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
        self.state = 0 # the state handler variable. 0: star, 1: chase, 2: 
                        #lookout
        self.last_state = 0 # records the previous state to see if states have 
                            #changed
        self.timepost = 0 # the time lookout() state change in ms
        self.buffer = 500 # the time of a lookout() swing in ms
        self.proximity = 1 # the distance of detecting to chase in m
        self.closest_dist = 0; # the distance of a person in range
        self.closest_dist_rad = 0.0; # the angle of a person in range

        # pubs & subs
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.get_scan, 10)
        
        # Run threads
        self.create_timer(0.1, self.run_threads)

    def run_threads(self):
        """runs the state and loop threads on a timer"""
        self.run_loop_thread = Thread(target=self.run_loop())
        self.run_state_handler_thread = Thread(target=self.state_handler())
        self.run_state_handler_thread.start()
        self.run_loop_thread.start()

    def run_loop(self):
        """run the robot states when there is a state change."""
        
        if self.state == 1 or self.state != self.last_state:
            self.last_state = self.state
            # run the current state
            match self.state:
                case 0:
                    msg = Twist()
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)

                    self.star()
                case 1:
                    self.chase()
                case 2:
                    msg = Twist()
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)
                    self.lookout()
        
    def state_handler(self): 
        """Handles transitions between states"""
        # find the closest lidar distance
        lidar_dist = self.closest_dist

        # transitions between FSM
        if lidar_dist < self.proximity:
            self.state = 1
        elif (self.state == 1 and lidar_dist > self.proximity):
            self.state = 2
        elif (self.state == 2):
            self.state = 0


    def star(self): 
        """Makes the robot move in star pattern
            forward_speed: speed to move forward in m/s
            turn_speed: the speed in rad/s to turn
            edge_length: the lenght of the edge of the star pattern
            dt: stall time""" 
        # Parameters
        forward_speed = 0.25                 # m/s
        turn_speed    = 0.65                 # rad/s
        edge_length   = 0.6                  # meters per star edge
        turn_angle    = math.radians(144.0)  # 144° turn for star
        heading_deg   = 36                   # rotate so one point faces up
        dt            = 0.02                 

        def drive(lin, ang, duration_s):
            """Publish velocity (lin, ang) for duration_s seconds, then stop.
            """
            msg = Twist()
            msg.linear.x  = lin
            msg.angular.z = ang
            end_time = time.time() + duration_s
            while self.state == 0 and time.time() < end_time:
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
            if (self.state == 0):
                drive(forward_speed, 0.0, edge_length / forward_speed)
                drive(0.0, turn_speed, turn_angle / turn_speed)

    def chase(self): 
        """uses a simple weighted distance equation to move towards an object
        or individual.
            Kp: the max forward speed to be scaled
            max_turn_speed: the max speed to to turn rad/s
            dt: the stall time
            target_distance: the distance at which the robot stops
            """
            
        # Makes robot chase a person
         # Parameters
        max_turn_speed    = 0.1                # rad/s
        dt = .02                                # sleep time
        Kp = .4                                 # max lin speed
        target_distance = .25                   # where robot stops

        msg = Twist()
        msg.linear.x = Kp*(self.closest_dist - target_distance)
        msg.angular.z = max_turn_speed * (self.closest_dist_rad)
        self.vel_pub.publish(msg)
        time.sleep(dt)

    def lookout(self): 
        """Turns the robot 120 degrees left and right to make the robot look
        confused after losing the individual being chased.
            turn_speed: the speed in m/s to turn
            turn_angle: the amount for the robot to turn (120 degrees) in 
                        radians.
            dt: the stall time"""
        # Parameters
        turn_speed = 0.35                 # rad/s
        turn_angle = math.radians(120.0)  # 120° in radians
        dt         = 0.02                 # sleep time 

        def turn(ang_vel, duration_s):
            """Publish angular velocity for duration_s seconds, then stop."""
            msg = Twist()
            msg.angular.z = ang_vel
            end_time = time.time() + duration_s
            while time.time() < end_time:
                self.vel_pub.publish(msg)
                time.sleep(dt)
            # stop
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            time.sleep(0.1)

        # Pan sequence
        # Left 120°
        turn(turn_speed, turn_angle / turn_speed)

        # Right 120° (negative angular velocity)
        turn(-turn_speed, turn_angle / turn_speed)

    def get_scan(self, msg):
        """runs when a lidar Twist() struct is published to the scan/ topic

        Runs upon a lidar message received. Filters out points below the lower
        bound and above the upper bound (upper_post). Then averages all the
        points not filtered to create a good guess on where a person/object
        exists in space before pushing that to an instance variable.
            lower_post: the lower filter cutoff
            upper_post: the upper filter cutoff
            filtered_dist_av: the variable storing the unfiltered distance
                                total.
            filtered_ang_av: the variable storing the unfiltered angle total.
            average_dividend: the amount to divide filtered_dist_av and
                            filtered_ang_av to become an actual average."""
        # len(msg.ranges) = 361
        lower_post = .25 #distance in meters to filter below
        upper_post = self.proximity #distance in meters to filter above
        filtered_dist_av = 0; #point cluster total distance to be averaged
        filtered_ang_av = 0; #point cluster total angle to be averaged
        average_dividend = 0;
        for i in range(len(msg.ranges) - 1):
            if lower_post < msg.ranges[i] and msg.ranges[i] < upper_post:
                filtered_dist_av = filtered_dist_av + msg.ranges[i]
                filtered_ang_av = filtered_ang_av + i 
                average_dividend = average_dividend + 1
        
        if(average_dividend != 0):
            self.closest_dist = filtered_dist_av / average_dividend
            self.closest_dist_rad = ((filtered_ang_av / average_dividend)/360) * (2 * math.pi)
        else:
            self.closest_dist = self.proximity + 1
            self.closest_dist_rad = 0

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    rclpy.spin(node)
    rclpy.shutdown()
