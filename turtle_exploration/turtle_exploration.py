import math
import rclpy
import sys

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
    ROTATING = 1
    DRIVING = 2
    CORNER = 3
    OBSTRUCTED = 4

class TurtleExploration(Node):
    def __init__(self):
        super().__init__('turtle_exploration')
        self.subscription = self.create_subscription(LaserScan,'scan', self.listener_callback, 10)
        self.scan = LaserScan

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speed = Vector3()
        self.rotation = Vector3()
        self.total_vel = Twist()

        self.state = State.INIT
        self.radius_of_robot = 0.15 #roughly
        self.shortest_range_in_front_of_robot = float('inf')
        self.longest_range_in_front_of_robot = 0
        self.num_ranges = 360
        self.index_of_angle = 0 #used for debug
        self.increment_of_scanner = 2*math.pi/self.num_ranges
        self.colliding = False
        self.collision_buffer = 0.5
        self.wall_to_hug = 'NaN'
        self.dist_from_hugging_wall = 0
        self.desired_dist_from_wall = 0.65
        self.desired_angle_on_wall = 0 #angle on wall: -1 negative, 0 zero, 1 positive
        self.angle_on_wall = 0
        self.search_speed = 0.3
        self.search_rotation_speed = 0.45
        self.wall_lost = False
    
    
    def set_shortest_and_longest_range_in_front_of_robot(self): #Should be split up in more functions
        all_ranges = self.scan.ranges
        quarter_of_ranges = self.num_ranges//4
        interesting_ranges_left = all_ranges[0:quarter_of_ranges]
        interesting_ranges_right = all_ranges[3*quarter_of_ranges:self.num_ranges-1]
        shortest_range_in_front_of_robot = float('inf')
        longest_range_in_front_of_robot = 0
    
        for i, range in enumerate(interesting_ranges_left):
            angle = self.increment_of_scanner*i
            thresh = self.radius_of_robot/(math.cos(math.pi/2-angle-self.radius_of_robot))
            if range < thresh:
                if range < shortest_range_in_front_of_robot:
                    shortest_range_in_front_of_robot = range
                    found_angle = angle
                if range > longest_range_in_front_of_robot and range != float('inf'):
                    longest_range_in_front_of_robot = range

        for i, range in enumerate(interesting_ranges_right):
            angle = self.increment_of_scanner*i + 3*math.pi/2
            thresh = self.radius_of_robot/(math.cos(3*math.pi/2-angle+self.radius_of_robot))
            if range < thresh:
                if range < shortest_range_in_front_of_robot:
                    shortest_range_in_front_of_robot = range
                    found_angle = angle
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
        if self.dist_from_hugging_wall == float('inf'):
            self.desired_angle_on_wall = 0
        elif self.wall_to_hug == 'left':
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
        offset_from_middle_of_robot = 30 #Checks for wall some angle in front of robot
        ranges = []
        #is_wall_lost = False
        buff = 0.1
        #define 3 angles to check
        if self.wall_to_hug == 'left':
            range = self.scan.ranges[self.num_ranges//4-offset_from_middle_of_robot]
        elif self.wall_to_hug == 'right':
            range = self.scan.ranges[3*self.num_ranges//4+offset_from_middle_of_robot]

        if range > self.desired_dist_from_wall + buff:
            self.wall_lost = True
        else:
            self.wall_lost = False
   
    
    def set_angle_on_wall(self):
        offset_from_middle_of_robot = 10
        thresh_for_angle = 0.03
        if self.wall_to_hug == 'left':
            angle1 = self.scan.ranges[self.num_ranges//4-offset_from_middle_of_robot]
            angle2 = self.scan.ranges[self.num_ranges//4+offset_from_middle_of_robot]
            diff = angle1 - angle2
            if thresh_for_angle < diff:
                self.angle_on_wall = -1
            elif -thresh_for_angle > diff:
                self.angle_on_wall = 1
            else:
                self.angle_on_wall = 0
        
        elif self.wall_to_hug == 'right':
            angle1 = self.scan.ranges[3*self.num_ranges//4+offset_from_middle_of_robot]
            angle2 = self.scan.ranges[3*self.num_ranges//4-offset_from_middle_of_robot]
            diff = angle1 - angle2
            if thresh_for_angle < diff:
                self.angle_on_wall = 1
            elif -thresh_for_angle > diff:
                self.angle_on_wall = -1
            else:
                self.angle_on_wall = 0
    
    
    def bug_algorithm_fsm(self):
        if self.state == State.INIT:
            self.find_closest_wall()
            self.state = State.DRIVING
        if self.state == State.DRIVING:
            self.collision_buffer = 0.8 
            self.set_speed(self.search_speed)
            self.set_dist_from_hugging_wall()
            self.decide_angle_on_wall()
            self.set_angle_on_wall()
            self.check_for_lost_wall()
            self.set_shortest_and_longest_range_in_front_of_robot()
            self.check_for_collision()
            if self.colliding == True:
                self.state = State.OBSTRUCTED
            if self.wall_lost == True:
                self.state = State.CORNER
            if self.desired_angle_on_wall == self.angle_on_wall:
                self.set_rotation(0.0)
            elif self.desired_angle_on_wall > self.angle_on_wall:
                self.set_rotation(self.search_speed)
            elif self.desired_angle_on_wall < self.angle_on_wall:
                self.set_rotation(-self.search_speed)
        if self.state == State.CORNER:
            self.check_for_lost_wall()
            self.set_shortest_and_longest_range_in_front_of_robot()
            self.check_for_collision()
            self.collision_buffer = 0.4
            if self.wall_to_hug == 'left':
                self.set_rotation(self.search_rotation_speed) #Follow curvature of corner
            elif self.wall_to_hug == 'right':
                self.set_rotation(-self.search_rotation_speed)
            if self.wall_lost == False:
                self.state = State.DRIVING
            if self.colliding == True:
                self.state = State.OBSTRUCTED
        if self.state == State.OBSTRUCTED:
            self.set_speed(0.0)
            self.check_for_lost_wall()
            self.set_shortest_and_longest_range_in_front_of_robot()
            self.check_for_collision()
            if self.wall_to_hug == 'left':
                self.set_rotation(-self.search_rotation_speed)
            elif self.wall_to_hug == 'right':
                self.set_rotation(self.search_rotation_speed)
            if self.colliding == False: #and self.wall_lost == False:
                self.state = State.DRIVING
    
        
    def listener_callback(self, msg):
        self.scan = msg
        self.num_ranges = len(msg.ranges)
        self.increment_of_scanner = 2*math.pi/self.num_ranges

    def timer_callback(self):
        self.bug_algorithm_fsm()
        self.publisher.publish(self.total_vel)

        self.get_logger().info('state %d' % self.state)
        self.get_logger().info('distance from hugging wall %f' % self.dist_from_hugging_wall)
        self.get_logger().info('hugging wall %s' % self.wall_to_hug)
        self.get_logger().info('wall lost %d' % self.wall_lost)
        self.get_logger().info('distance in front %f' % self.shortest_range_in_front_of_robot)
        if self.colliding == True:
            self.get_logger().info('COLLIDING')
        self.get_logger().info('\n')


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
