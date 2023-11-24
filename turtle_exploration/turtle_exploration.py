import math
import rclpy
import sys
import tf2_geometry_msgs
import std_msgs

from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

class State():
    INIT = 0
    IDLE = 1
    ROTATING = 2
    DRIVING = 3
    CORNER = 4 #Bug algorithm
    OBSTRUCTED = 5

class TurtleExploration(Node):
    def __init__(self):
        super().__init__('turtle_exploration')
        self.subscription = self.create_subscription(LaserScan,'scan', self.listener_callback, 10)
        self.scan = LaserScan

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speed = Vector3()
        self.speed_x = 0.0
        self.rotation = Vector3()
        self.rotation_z = 0.0
        self.total_vel = Twist()

        self.state = State.INIT
        
        self.width_of_robot = 0.3 #roughly
        self.shortest_range_in_front_of_robot = float('inf')
        self.longest_range_in_front_of_robot = 0
        self.angle_of_longest_dist = 0
        self.range_of_longest_dist = 0
        self.num_ranges = 360
        self.index_of_angle = 0 #used for debug
        self.increment_of_scanner = 2*math.pi/self.num_ranges
        self.colliding = False
        self.collision_buffer = 0.8 #meters
        #bug algorithm
        self.wall_to_hug = 'NaN'
        self.dist_from_hugging_wall = 0
        self.desired_dist_from_wall = 0.6
        #angle on wall: -1 negative, 0 zero, 1 positive
        self.desired_angle_on_wall = 0
        self.angle_on_wall = 0
        self.diff = 0 #for debug
        self.search_speed = 0.5
        self.wall_lost = False
        
        #TF
        # self.target_frame = self.declare_parameter('target_frame', 'base_link').get_parameter_value().string_value
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def set_angle_and_dist_of_longest_defined_dist(self):
        longest_range = 0
        longest_angle = 0
        index = 0
        angle_rad = 0

        for i, element in enumerate(self.scan.ranges):
            angle_rad = self.increment_of_scanner*i
            #Exclude angles behind robot
            if not(math.pi/2 < angle_rad < 3*math.pi/2) and element != float('inf') and (element > longest_range):
                longest_range = element
                longest_angle = angle_rad
                index = i
            
        self.index_of_angle = index
        self.range_of_longest_dist = longest_range 
        self.angle_of_longest_dist = longest_angle
    

    def set_shortest_and_longest_range_in_front_of_robot(self): #Should be split up in more functions
        all_ranges = self.scan.ranges
        quarter_of_ranges = self.num_ranges//4
        interesting_ranges_left = all_ranges[0:quarter_of_ranges]
        interesting_ranges_right = all_ranges[3*quarter_of_ranges:self.num_ranges-1]
        shortest_range_in_front_of_robot = float('inf')
        longest_range_in_front_of_robot = 0

        for i, range in enumerate(interesting_ranges_left):
            angle = self.increment_of_scanner*i
            thresh = self.width_of_robot/(2*math.cos(math.pi/2-angle))
            if range < thresh:
                if range < shortest_range_in_front_of_robot:
                    shortest_range_in_front_of_robot = range
                if range > longest_range_in_front_of_robot and range != float('inf'):
                    longest_range_in_front_of_robot = range

        for i, range in enumerate(interesting_ranges_right):
            angle = self.increment_of_scanner*i
            thresh = self.width_of_robot/(2*math.cos(angle))
            if range < thresh:
                if range < shortest_range_in_front_of_robot:
                    shortest_range_in_front_of_robot = range
                if range > longest_range_in_front_of_robot and range != float('inf'):
                    longest_range_in_front_of_robot = range

        self.shortest_range_in_front_of_robot = shortest_range_in_front_of_robot
        self.longest_range_in_front_of_robot = longest_range_in_front_of_robot
 
    def check_for_collision(self):
        thresh_broken = False
        if self.shortest_range_in_front_of_robot < self.collision_buffer:
            thresh_broken = True
        if thresh_broken == True:
            self.colliding = True
        else:
            self.colliding = False

    def set_rotation(self, angular_vel):
        self.rotation.x = 0.0
        self.rotation.y = 0.0
        self.rotation.z = angular_vel
        self.rotation_z = angular_vel
        self.total_vel.angular = self.rotation

    def set_speed(self, linear_vel):
        self.speed.x = linear_vel
        self.speed.y = 0.0
        self.speed.z = 0.0
        self.speed_x = linear_vel
        self.total_vel.linear = self.speed
    
    def find_closest_wall(self):
        if self.scan.ranges[self.num_ranges//4] < self.scan.ranges[3*self.num_ranges//4]:
            self.wall_to_hug = 'left'
        else:
            self.wall_to_hug = 'right'
    
    def set_dist_from_hugging_wall(self):
        if self.wall_to_hug == 'left':
            self.dist_from_hugging_wall = self.scan.ranges[self.num_ranges//4]
        elif self.wall_to_hug == 'right':
            self.dist_from_hugging_wall = self.scan.ranges[3*self.num_ranges//4]

    def decide_angle_on_wall(self):
        buffer = 0.1
        self.desired_angle_on_wall = 0
        if self.wall_to_hug == 'left':
            if self.dist_from_hugging_wall < self.desired_dist_from_wall-buffer:
                self.desired_angle_on_wall = -1
            elif self.dist_from_hugging_wall > self.desired_dist_from_wall+buffer:
                self.desired_angle_on_wall = 1
        elif self.wall_to_hug == 'right':
            if self.dist_from_hugging_wall < self.desired_dist_from_wall-buffer:
                self.desired_angle_on_wall = 1
            elif self.dist_from_hugging_wall > self.desired_dist_from_wall+buffer:
                self.desired_angle_on_wall = -1
          
    def check_for_lost_wall(self):
        offset_from_middle_of_robot = 5
        ranges = []
        wall_lost = False
        #define 3 angles to check
        if self.wall_to_hug == 'left':
            ranges = [self.scan.ranges[self.num_ranges//4]]
                    #   self.scan.ranges[self.num_ranges//4-offset_from_middle_of_robot], 
                    #   self.scan.ranges[self.num_ranges//4+offset_from_middle_of_robot]]
        elif self.wall_to_hug == 'right':
            ranges = [self.scan.ranges[3*self.num_ranges//4]] 
                    #   self.scan.ranges[3*self.num_ranges//4-offset_from_middle_of_robot], 
                    #   self.scan.ranges[3*self.num_ranges//4+offset_from_middle_of_robot]]
            
        for elem in ranges:
            if elem > self.desired_dist_from_wall + 0.5:
                wall_lost = True

        if wall_lost == True:
            self.wall_lost = True
        else:
            self.wall_lost = False
    

    def set_angle_on_wall(self):
        offset_from_middle_of_robot = 25
        thresh_for_angle = 0.1
        if self.wall_to_hug == 'left':
            angle1 = self.scan.ranges[self.num_ranges//4-offset_from_middle_of_robot]
            angle2 = self.scan.ranges[self.num_ranges//4+offset_from_middle_of_robot]
            self.diff = angle1 - angle2
            if thresh_for_angle < self.diff:
                self.angle_on_wall = -1
            elif -thresh_for_angle > self.diff:
                self.angle_on_wall = 1
            else:
                self.angle_on_wall = 0
        
        elif self.wall_to_hug == 'right':
            angle1 = self.scan.ranges[3*self.num_ranges//4+offset_from_middle_of_robot]
            angle2 = self.scan.ranges[3*self.num_ranges//4-offset_from_middle_of_robot]
            self.diff = angle1 - angle2
            if thresh_for_angle < self.diff:
                self.angle_on_wall = 1
            elif -thresh_for_angle > self.diff:
                self.angle_on_wall = -1
            else:
                self.angle_on_wall = 0
    
    


    def bug_algorithm_fsm(self):
        if self.state == State.INIT:
            self.find_closest_wall()
            self.state = State.DRIVING
            self.set_speed(self.search_speed)
        #if self.state == State.IDLE:
        if self.state == State.DRIVING:

            self.set_dist_from_hugging_wall()
            self.decide_angle_on_wall()
            self.set_angle_on_wall()
            self.check_for_lost_wall()
            if self.desired_angle_on_wall == self.angle_on_wall:
                self.set_rotation(0.0)
            elif self.desired_angle_on_wall > self.angle_on_wall:
                self.set_rotation(self.search_speed)
            elif self.desired_angle_on_wall < self.angle_on_wall:
                self.set_rotation(-self.search_speed)
            if self.wall_lost == True:
                self.state = State.CORNER
        if self.state == State.CORNER:
            self.check_for_lost_wall()
            if self.wall_to_hug == 'left':
                self.set_speed(self.search_speed)
                self.set_rotation(self.search_speed/self.desired_dist_from_wall) #Follow curvature of corner
            elif self.wall_to_hug == 'right':
                self.set_rotation(-self.search_speed/self.desired_dist_from_wall)
            if self.wall_lost == False:
                self.state = State.DRIVING
                
            

    
    def exploration_fsm(self):
        # init = 0
        # idle = 1
        # rotating = 2
        # driving = 3
        # obstructed = 5
        self.set_shortest_and_longest_range_in_front_of_robot()
        self.check_for_collision()

        if self.state == State.INIT:
            self.set_angle_and_dist_of_longest_defined_dist()
            self.state = State.IDLE
        elif self.state == State.IDLE:
            self.set_angle_and_dist_of_longest_defined_dist()
            self.state = State.ROTATING
        elif self.state == State.ROTATING:
            angle = self.angle_of_longest_dist
            dist = self.range_of_longest_dist
            if angle > 0 and angle < 3*math.pi/4:   
                self.set_rotation([0.0,0.0,0.5])
            elif angle > 5*math.pi/4 and angle < 2*math.pi:
                self.set_rotation([0.0,0.0,-0.5])
            else:
                self.state = State.DRIVING  
                self.set_rotation([0.0,0.0,0.0])
            if self.longest_range_in_front_of_robot < dist+0.2 and self.longest_range_in_front_of_robot > dist-0.2 and self.shortest_range_in_front_of_robot > self.collision_buffer:
                self.set_rotation([0.0,0.0,0.0])
                self.state = State.DRIVING
            if self.shortest_range_in_front_of_robot <= 1.0:
                self.set_angle_and_dist_of_longest_defined_dist()
        elif self.state == State.DRIVING:
            if self.shortest_range_in_front_of_robot < 0.3:
                self.set_speed([-0.5,0.0,0.0])
            else:
                self.set_speed([0.5,0.0,0.0])
            if self.colliding == True:
                self.state = State.OBSTRUCTED
        elif self.state == State.OBSTRUCTED:
            self.set_speed([0.0,0.0,0.0])
            
            self.state = 1
    
        
    def listener_callback(self, msg):
        self.scan = msg
        self.num_ranges = len(msg.ranges)
        self.increment_of_scanner = 2*math.pi/self.num_ranges
        #translation, rotation = self.pose_in_map_frame()
        #self.get_logger().info('I heard: %f, %f' % self.angle_and_dist_of_longest_defined_dist(self.scan))
        #print(self.angle_and_dist_of_longest_defined_dist(msg))

    def timer_callback(self):
        self.bug_algorithm_fsm()
        self.publisher.publish(self.total_vel)
        self.get_logger().info('state %d' % self.state)
        self.get_logger().info('distance from hugging wall %f' % self.dist_from_hugging_wall)
        self.get_logger().info('hugging wall %s' % self.wall_to_hug)
        self.get_logger().info('wall lost %d' % self.wall_lost)
        self.get_logger().info('desired angle on wall %s' % self.desired_angle_on_wall)
        self.get_logger().info('angle on wall %s' % self.angle_on_wall)
        # self.get_logger().info('shortest range in front of robot %f' % self.shortest_range_in_front_of_robot)
        # self.get_logger().info('longest range in front of robot %f' % self.longest_range_in_front_of_robot)
        # self.get_logger().info('longest defined range %f' % self.range_of_longest_dist)
        # self.get_logger().info('angle of longest range %f' % self.angle_of_longest_dist)
        # self.get_logger().info('index of angle %d' % self.index_of_angle)
        if self.colliding == True:
            self.get_logger().info('COLLIDING')


def main(args=None):
    
    rclpy.init(args=args)
    
    try:
        turtle_exploration = TurtleExploration()
        rclpy.spin(turtle_exploration)
    except BaseException:
        print('Exception in route_manager:', file=sys.stderr)
        raise
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
