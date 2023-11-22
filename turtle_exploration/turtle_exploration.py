import math
import rclpy
import sys
import tf2_geometry_msgs

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
        
        self.width_of_robot = 0.2 #roughly
        self.shortset_range_in_front_of_robot = float('inf')
        self.angle_of_longest_dist = 0
        self.range_of_longest_dist = 0
        self.num_ranges = 360
        self.index_of_angle = 0 #used for debug
        self.increment_of_scanner = 2*math.pi/self.num_ranges
        self.colliding = False
        self.collision_buffer = 0.5 #meters

        
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
    
    def set_range_in_front_of_robot(self):
        #Checks in front of robot for defined range
        found = False
        index = 0
        while not found and index < self.num_ranges-1:
            if self.scan.ranges[index] != float('inf'):
                self.range_in_front_of_robot = self.scan.ranges[index]
                found = True
            if not found and self.scan.ranges[self.num_ranges-index-1] != float('inf'): #Checks for range right of the robot, 360 ranges
                self.range_in_front_of_robot = self.scan.ranges[self.num_ranges-index-1]
                found = True
            index += 1

    def set_shortest_range_in_front_of_robot(self):
        all_ranges = self.scan.ranges
        quarter_of_ranges = self.num_ranges//4
        interesting_ranges_left = all_ranges[0:quarter_of_ranges]
        interesting_ranges_right = all_ranges[3*quarter_of_ranges:self.num_ranges-1]
        shortest_range = float('inf')

        for i, range in enumerate(interesting_ranges_left):
            angle = self.increment_of_scanner*i
            if i == 0: #Avoid deviding by 0
                thresh = 3.5
            else: 
                thresh = self.width_of_robot/(2*math.cos(math.pi/2-angle))
            range_in_front_of_robot = float('inf')
            if range < thresh:
                range_in_front_of_robot = range
            if range_in_front_of_robot < shortest_range:
                shortest_range = range_in_front_of_robot

        for i, range in enumerate(interesting_ranges_right):
            angle = self.increment_of_scanner*i
            thresh = self.width_of_robot/(2*math.cos(angle))
            range_in_front_of_robot = float('inf')
            if range < thresh:
                range_in_front_of_robot = range
            if range_in_front_of_robot < shortest_range:
                shortest_range = range_in_front_of_robot

        self.shortest_range_in_front_of_robot = shortest_range

    
    def check_for_collision(self):
        thresh_broken = False
        if self.shortest_range_in_front_of_robot < self.collision_buffer:
            thresh_broken = True
        if thresh_broken == True:
            self.colliding = True
        else:
            self.colliding = False


    def set_rotation(self, angular_vel):
        self.rotation.x = angular_vel[0]
        self.rotation.y = angular_vel[1]
        self.rotation.z = angular_vel[2]
        self.total_vel.angular = self.rotation

    def set_speed(self, linear_vel):
        self.speed.x = linear_vel[0]
        self.speed.y = linear_vel[1]
        self.speed.z = linear_vel[2]
        self.total_vel.linear = self.speed
    
    def exploration_fsm(self):
        # init = 0
        # idle = 1
        # rotating = 2
        # driving = 3
        # obstructed = 4
        self.set_shortest_range_in_front_of_robot()
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
            #self.set_range_in_front_of_robot()
            if angle > 0 and angle < 3*math.pi/4:   
                self.set_rotation([0.0,0.0,0.5])
            elif angle > 5*math.pi/4 and angle < 2*math.pi:
                self.set_rotation([0.0,0.0,-0.5])
            if self.shortest_range_in_front_of_robot < dist+0.2 and self.shortest_range_in_front_of_robot > dist-0.2:
                self.set_rotation([0.0,0.0,0.0])
                self.state = State.DRIVING
            if self.shortest_range_in_front_of_robot <= 1.0:
                self.set_angle_and_dist_of_longest_defined_dist()
            #self.set_range_in_front_of_robot()
        elif self.state == State.DRIVING:
            self.set_speed([0.6,0.0,0.0])
            if self.colliding == True:
                self.state = State.OBSTRUCTED
        elif self.state == State.OBSTRUCTED:
            self.set_speed([0.0,0.0,0.0])
            self.state = 1
            

    # def pose_in_map_frame(self):
    #     from_frame = self.target_frame
    #     to_frame = 'map'
    #     t = TransformStamped
    #     try:
    #         t = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
    #     except TransformException as ex:
    #                 self.get_logger().info(
    #                     f'Could not transform {to_frame} to {from_frame}: {ex}')
        
    #     euler_angles = self.quat_to_rpy(t.transform.rotation)
    #     return t.transform.translation, euler_angles

    # def quat_to_rpy(self, quaternion):
    #     x = quaternion.x
    #     y = quaternion.y
    #     z = quaternion.z
    #     w = quaternion.w
        
    #     sinr_cosp = 2 * (w * x + y * z)
    #     cosr_cosp = 1 - 2 * (x * x + y * y)
    #     roll = math.atan2(sinr_cosp, cosr_cosp)

    #     sinp = 2 * (w * y - z * x)
    #     pitch = math.asin(sinp)

    #     siny_cosp = 2 * (w * z + x * y)
    #     cosy_cosp = 1 - 2 * (y * y + z * z)
    #     yaw = math.atan2(siny_cosp, cosy_cosp)

    #     return roll, pitch, yaw
    
        
    def listener_callback(self, msg):
        self.scan = msg
        self.num_ranges = len(msg.ranges)
        self.increment_of_scanner = 2*math.pi/self.num_ranges
        #translation, rotation = self.pose_in_map_frame()
        #self.get_logger().info('I heard: %f, %f' % self.angle_and_dist_of_longest_defined_dist(self.scan))
        #print(self.angle_and_dist_of_longest_defined_dist(msg))

    def timer_callback(self):
        self.exploration_fsm()
        self.publisher.publish(self.total_vel)
        self.get_logger().info('state %d' % self.state)
        
        self.get_logger().info('range in front of robot %f' % self.shortest_range_in_front_of_robot)
        self.get_logger().info('longest defined range %f' % self.range_of_longest_dist)
        self.get_logger().info('angle of longest range %f' % self.angle_of_longest_dist)
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
