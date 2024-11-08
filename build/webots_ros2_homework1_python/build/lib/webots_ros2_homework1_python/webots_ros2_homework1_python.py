import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math



LINEAR_VEL = 0.22
STOP_DISTANCE = 0.1
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.45
LIDAR_SEEK_DISTANCE = 0.65
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90
MOVE_SPEED = 0.24
TURN_SPEED = 0.2

class RightHandRule(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        #self.right_hand = False
        #self.x_bounds = []
        #self.y_bounds = []
        #self.front_bounds = []
        #self.bound_count = 0
        #self.front_count = 0
        self.gpsx_bounds = []
        self.gpsy_bounds = []
        self.gps_count = 0
        self.avoid_obst_count = 0
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber3 = self.create_subscription(
            PointStamped,
            '/TurtleBot3Burger/gps',
            self.listener_callback3,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.3
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
            	self.scan_cleaned.append(reading)



    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz))
        # similarly for twist message if you need
        self.pose_saved=position

        #self.bound_count += 1
        #if len(self.x_bounds) < 2 or posx < self.x_bounds[0] or posx > self.x_bounds[1]:
        #    self.x_bounds = [posx - .5, posx + 0.5]
        #    self.bound_count = 0
        #if len(self.y_bounds) < 2 or posy < self.y_bounds[0] or posy > self.y_bounds[1]:
        #    self.y_bounds = [posy - .5, posy + 0.5]
        #    self.bound_count = 0
        #self.get_logger().info('loops since re-bounded: {}'.format(self.bound_count))
        
        #Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        #diffX = math.fabs(self.pose_saved.x- position.x)
        #diffY = math.fabs(self.pose_saved.y - position.y)
        #if (diffX < 0.0001 and diffY < 0.0001):
           #self.stall = True
        #else:
           #self.stall = False
           
        return None

    def listener_callback3(self, msg3):
        gps_pos = msg3.point
        (gpsx, gpsy, gpsz) = (gps_pos.x, gps_pos.y, gps_pos.z)
        self.get_logger().info('gps coordinates: {}, {}, {}'.format(gpsx, gpsy, gpsz))

        self.gps_count += 1
        if len(self.gpsx_bounds) != 2 or gpsx < self.gpsx_bounds[0] or gpsx > self.gpsx_bounds[1]:
            self.gps_count = 0
            self.gpsx_bounds = [gpsx - 0.08, gpsx + 0.08]
        if len(self.gpsy_bounds) != 2 or gpsy < self.gpsy_bounds[0] or gpsy > self.gpsy_bounds[1]:
            self.gps_count = 0
            self.gpsy_bounds = [gpsy - 0.08, gpsy + 0.08]
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
    	    self.turtlebot_moving = False
    	    return
    	    
        #left_lidar_samples = self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX]
        #right_lidar_samples = self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]
        #front_lidar_samples = self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]
        
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        front_dist = min(self.scan_cleaned[160:200])
        avoid_front = front_dist < LIDAR_AVOID_DISTANCE
        right_dist = min(self.scan_cleaned[255:285])
        avoid_right = right_dist < LIDAR_AVOID_DISTANCE
        seek_dist = min(self.scan_cleaned[270:320])
        seek_right = seek_dist < LIDAR_SEEK_DISTANCE
        #self.get_logger().info('[front: {}, right: {}, seek: {}]'.format(front_dist, right_dist, seek_dist))

        #self.front_count += 1
        #if len(self.front_bounds) < 2 or front_lidar_min < self.front_bounds[0] or front_lidar_min > self.front_bounds[1]:
        #    self.front_bounds = [front_lidar_min - .1, front_lidar_min + .1]
        #    self.front_count = 0
        
        #self.get_logger().info('loops since front re-bounded: {}'.format(self.front_count))
        #self.get_logger().info('loops since loc re-bounded: {}'.format(self.bound_count))
        #if self.front_count > 30 or self.bound_count > 150:
        #    self.avoid_obst_count = 30
        #    self.front_count = 0
        #    self.bound_count = 0

        self.get_logger().info('loops since gps re-bounded: {}'.format(self.gps_count))
        if self.gps_count > 90:
            self.avoid_obst_count = 36
            self.gps_count = 0

        #self.get_logger().info('left scan slice: "%s"'%  min(left_lidar_samples))
        #self.get_logger().info('front scan slice: "%s"'%  min(front_lidar_samples))
        #self.get_logger().info('right scan slice: "%s"'%  min(right_lidar_samples))

        if front_lidar_min < SAFE_STOP_DISTANCE:
            if self.turtlebot_moving == True:
                self.cmd.linear.x = -0.1
                self.cmd.angular.z = 0.2 
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Reversing')
                return
        elif self.avoid_obst_count > 30:
            self.avoid_obst_count -= 1
            self.cmd.linear.x = -MOVE_SPEED 
            self.cmd.angular.z = 0.0 
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info('Stuck - reversing')
            self.gps_count = 0
            return
        elif self.avoid_obst_count > 14:
            self.avoid_obst_count -= 1
            self.cmd.linear.x = 0.0 
            self.cmd.angular.z = TURN_SPEED 
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info('Stuck - turning left')
            self.gps_count = 0
            return
        elif self.avoid_obst_count > 4:
            self.avoid_obst_count -= 1
            self.cmd.linear.x = MOVE_SPEED 
            self.cmd.angular.z = 0.0 
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info('Stuck - advancing')
            self.gps_count = 0
            return
        elif self.avoid_obst_count > 0:
            self.avoid_obst_count -= 1
            self.cmd.linear.x = 0.0 
            self.cmd.angular.z = -0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info('Stuck - waiting')
            self.gps_count = 0
            return
        elif avoid_front:
            self.cmd.linear.x = 0.0 
            self.cmd.angular.z = 0.3 
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info('Turning left')
            return
        elif not avoid_right and seek_right:
            self.cmd.linear.x = 0.0 
            self.cmd.angular.z = -0.3 
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.get_logger().info('Turning right')
            return
        else: # not avoid_front and (avoid_right or not seek_right):
            self.cmd.linear.x = MOVE_SPEED 
            self.cmd.angular.z = -0.18 
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            if avoid_right:
                self.right_hand = True
            self.get_logger().info('Moving straight/seeking')
            return
            

        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
           self.get_logger().info('Stall reported')
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)
 


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RightHandRule()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
