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
        # global variables
        int state = 0 # the state handler variable
        int timepost = 0 # the time lookout() state change in ms
        int buffer = 500 # the time of a lookout() swing in ms
        float proximity = 1 # the distance of detecting to chase in m
        
    def run_loop(self):
        """run the robot states"""
        # check transitions
        self.state_handler()

        # run the current state
        switch(state):
            case 0:
                star()
                break
            case 1:
                chase()
                break
            case 2:
                lookout()
                break
        
    def state_handler(self): 
        """Handles transitions between states"""
        # find the closest lidar distance
        lidar_dist  = #DO THIS

        # find the current time
        current_time = #DO THIS

        # transitions between FSM
        if lidar_dist < proximity:
            state = 1
        elif (state == 1 && lidar_dist > proximity):
            state = 2
            timepost = #current time goes here
        elif (state == 2 && current_time >= timepost + (buffer * 3))
            state = 0


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
