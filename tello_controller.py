#!/usr/bin/env python2

import sys
import time
import rospy
import numpy as np

from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Empty

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from tello_driver.msg import TelloStatus

MAX_POS_ERROR = 0.1
BASE_SPEED_LEADER = 0.5 #orig = 0.5
BASE_SPEED_FOLLOWER = 1
SPEED_STOP_ERROR = 0.07


class TelloController():
    '''
        Single Tello EDU Controller

        Contains utility functions for
          - Takeoff
          - Land
          - Navigate to position

        For navigation:
          - Uses onboard odometry if no external positioning is given.
          - Relies on pose published to "pos_topic" otherwise.
            (currently only for position)
    '''

    def __init__(self, i, ros_ns="tello", pos_topic=None, takeoff_height=1.2, external_z=True) :
        '''
            Controller to be used by TelloSwarm
            (can also be used standalone)

            Params:
              - takeoff_height: default takeoff
              - external_z: use external pos for height (if true) or odom
        '''

        self.id = i
        self.ns = ros_ns

        self.x0 = None
        self.y0 = None
        self.z0 = None

        self.x = None
        self.y = None
        self.z = None

        self.roll = None

        self.pos = np.array([None, None, None])
        self.obj = np.array([None, None, takeoff_height])
        self.obj_path = None

        self.vel = Twist()

        self.diff = None # the positioning distance error

        self.external_z = external_z

        self.running = False
        self.connection = False
        self.battery = -1

        self.land_pub = rospy.Publisher('/{}/land'.format(self.ns), Empty, queue_size=10)
        self.takeoff_pub = rospy.Publisher('/{}/takeoff'.format(self.ns), Empty, queue_size=10)
        self.emergency_pub = rospy.Publisher('/{}/emergency'.format(self.ns), Empty, queue_size=10)
        self.vel_pub = rospy.Publisher('/{}/cmd_vel'.format(self.ns), Twist, queue_size=10)


        self.odom_sub = rospy.Subscriber('/{}/odom'.format(self.ns), Odometry, self.cb_odom)
        self.tello_status = rospy.Subscriber('/{}/status'.format(self.ns), TelloStatus, self.cb_status)

        if pos_topic :
            print("Subscribing to {}".format(pos_topic))
            self.pos_topic = pos_topic
            # self.position_sub = rospy.Subscriber(self.pos_topic, PoseStamped, self.cb_positioning)
            self.position_sub = rospy.Subscriber(self.pos_topic, PoseStamped, self.currrent_positioning)
            # self.forward_sub = rospy.Subscriber(self.pos_topic, PoseStamped, self.forward)
            sys.stdout.write('Waiting for external position...')
            sys.stdout.flush()

        time.sleep(1)        
        i = 0
        rate = rospy.Rate(5)
        while i < 42 :
            sys.stdout.write('.')
            sys.stdout.flush()
            if self.x is not None and self.y is not None and self.z is not None :
                sys.stdout.write('... position locked!\n\n')
                sys.stdout.flush()
                rospy.loginfo('Tello controller for drone {} in network namespace {} is ready.'.format(self.id, self.ns))
                # verify if the drone is connected
                self.connection = True
                rospy.loginfo('Tello {} in network namespace {} is connected.'.format(self.id, self.ns))
                self.x0 = self.x
                self.y0 = self.y
                self.z0 = self.z
                rospy.loginfo('Initial position of tello{} is: [{},{}, {}]'.format(self.id, self.x0, self.y0, self.z0))
                self.obj_x = self.x
                self.obj_y = self.y
                break
            else :    
                rate.sleep()
                i += 1
         
        if i == 42 :
            sys.stdout.write('\n\n')
            sys.stdout.flush()
            rospy.loginfo('Tello controller for drone {} timed out.'.format(self.id))

    def is_connection(self):
        return self.connection

    def cb_odom(self, msg) :
        '''
            Tello odom subscriber.
            Only updates positions if there is no external ref.
        '''
        if not self.pos_topic :
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.z = msg.pose.pose.position.z

        if not self.external_z :
            self.z = msg.pose.pose.position.z
            self.set_speed()
    
    def cb_status(self, msg) :
        '''

        '''
        self.battery = msg.battery_percentage
    
    def cb_positioning(self, msg) :
        '''
            External positioning subscriber
        '''
        self.x = msg.pose.position.z
        self.y = msg.pose.position.x
        if self.external_z :
            self.z = msg.pose.position.y
        
        self.pos = np.array([self.x, self.y, self.z])

        if self.running and self.obj.all() and self.pos.all() :
            self.set_speed()

    def currrent_positioning(self, msg) :
        '''
            External positioning subscriber
        '''
        # H0 = np.array([[0,0,1,0], [1,0,0,0], [0,1,0,0],[0,0,0,1]])
        # position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        # R = 
        self.x = msg.pose.position.x
        self.y = msg.pose.position.z
        if self.external_z :
            self.z = msg.pose.position.y

        # quaternion = (
        #     msg.pose.orientation.x,
        #     msg.pose.orientation.y,
        #     msg.pose.orientation.z,
        #     msg.pose.orientation.w)
        
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler [2]
        
        self.pos = np.array([self.x, self.y, self.z])


    def set_objective(self, pos) :
        '''
            Sets objective position
            (TODO: add orientation)
        '''
        assert len(pos) == 3
        self.obj = np.array(pos)
        # self.obj_x = x#-self.x0
        # self.obj_y = y#-self.y0
        # self.obj_z = z

    def set_path(self, path) :
        '''
            Sets objective position
            (TODO: add orientation)
        '''
        # assert (path == None )
        self.obj_path = path
        # self.obj_x = x#-self.x0
        # self.obj_y = y#-self.y0
        # self.obj_z = z

    def set_speed(self) :
        '''
            Calculates speed to current objective position.

            Currently called from the subscriber of odom or position.
        '''

        # Message template
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        # Ensure no rotation    
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        diff = self.obj - self.pos
        dist = np.linalg.norm(diff)

        while (dist > SPEED_STOP_ERROR):
            # Calculate velocity vector
            # print(self.pos)
            # print(self.obj)
            diff = self.obj[:3] - self.pos[:3]
            print('diff of drone {}: {}\n'.format(self.id, diff))

            dist = np.linalg.norm(diff)
            speed_multiplier = BASE_SPEED
            if dist < MAX_POS_ERROR :
                speed_multiplier *= 0.3
            #[twist.linear.x, twist.linear.y] = speed_multiplier * diff / dist
            [twist.linear.x, twist.linear.y, twist.linear.z] = speed_multiplier * diff / dist

            # TO REMOVE. For now using in setup without stable "Z"
            #twist.linear.z = 0

            # Publish
            # rospy.loginfo(str(twist))
            self.vel_pub.publish(twist)
        
        # when the distance error belows the SPEED_STOP_ERROR, let drone hover
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        self.vel = twist
        # Publish
        # rospy.loginfo(str(twist))
        print(twist.linear.x)
        for _ in range(2) :
            self.vel_pub.publish(twist)
            time.sleep(0.2)

    def set_speed_step_leader(self, diff) :
        '''
            Calculates speed to current objective position.

            Currently called from the subscriber of odom or position.
        '''

        # Message template
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        # Ensure no rotation    
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        print('diff of drone {}: {}\n'.format(self.id, diff))

        dist = np.linalg.norm(diff)
        speed_multiplier = BASE_SPEED_LEADER

        # if the distance error below the max position error, slow down
        #if dist < MAX_POS_ERROR :
        #    speed_multiplier *= 0.3

        # if the distance error below the stop error, then hovering
        if dist < SPEED_STOP_ERROR:
            speed_multiplier = 0.0

        #[twist.linear.x, twist.linear.y] = speed_multiplier * diff / dist
        [twist.linear.x, twist.linear.y, twist.linear.z] = speed_multiplier * diff / dist
        #[twist.linear.x, twist.linear.y, twist.linear.z] = [0, 0, 0]
        self.vel = twist
        # [twist.linear.x, twist.linear.y, twist.linear.z] = speed_multiplier * diff
        #[twist.linear.x, twist.linear.y, twist.linear.z] = [0.2,	0,	0]
        # TO REMOVE. For now using in setup without stable "Z"
        #twist.linear.z = 0

        # Publish
        rospy.loginfo(str(twist))
        self.vel_pub.publish(twist)

    
    def set_speed_step_follower(self, diff) :
        '''
            Calculates speed to current objective position.

            Currently called from the subscriber of odom or position.
        '''

        # Message template
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        # Ensure no rotation    
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        print('diff of drone {}: {}\n'.format(self.id, diff))

        dist = np.linalg.norm(diff)
        speed_multiplier = BASE_SPEED_FOLLOWER

        # if the distance error below the max position error, slow down
        #if dist < MAX_POS_ERROR :
        #    speed_multiplier *= 0.3

        # if the distance error below the stop error, then hovering
        if dist < SPEED_STOP_ERROR:
            speed_multiplier = 0.0

        #[twist.linear.x, twist.linear.y] = speed_multiplier * diff / dist
        [twist.linear.x, twist.linear.y, twist.linear.z] = speed_multiplier * diff 
        self.vel = twist
        # [twist.linear.x, twist.linear.y, twist.linear.z] = speed_multiplier * diff
        #[twist.linear.x, twist.linear.y, twist.linear.z] = [0.2,	0,	0]
        # TO REMOVE. For now using in setup without stable "Z"
        #twist.linear.z = 0

        # Publish
        rospy.loginfo(str(twist))
        self.vel_pub.publish(twist)

    def set_speed(self, vel_linear) :
        '''
            Calculates speed to current objective position.

            Currently called from the subscriber of odom or position.
        '''

        # Message template
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        # Ensure no rotation    
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        [twist.linear.x, twist.linear.y, twist.linear.z] = vel_linear

        self.vel = twist

        # Publish
        rospy.loginfo(str(twist))
        self.vel_pub.publish(twist)
            

    def takeoff(self) :
        '''
            Publishes a takeoff message
            two times in 0.2s intervals.
        '''
        rospy.loginfo("Tello{} attempting take off...".format(self.id))
        msg = Empty()
        for _ in range(2) :
            self.takeoff_pub.publish(msg)
            time.sleep(0.2)

    def circle(self) :
        '''
            Moving forward
            two times in 0.2s intervals.
        '''
        # Message template
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0.4
        twist.linear.z = 0

        # speed_multiplier = BASE_SPEED_LEADER 
        # twist.linear.y = speed_multiplier
        rospy.loginfo('Initial position of tello{} is: [{},{}, {}], \n \
            Current Position of tello{} is: [{},{},{}]'.format(self.id, self.x0, self.y0, self.z0, self.id, self.x, self.y, self.z))

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0.6  ##smaller value 

        self.vel = twist

        self.vel_pub.publish(twist)
        time.sleep(0.2)

    def forward(self, pos) :
        '''
            Moving forward
            two times in 0.2s intervals.
        '''
        # Message template
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        speed_multiplier = BASE_SPEED
        twist.linear.y = speed_multiplier
        rospy.loginfo('Initial position of tello{} is: [{},{}, {}], \n \
            Current Position of tello{} is: [{},{},{}]'.format(self.id, self.x0, self.y0, self.z0, self.id, self.x, self.y, self.z))

        # Ensure no rotation    
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        rospy.loginfo("Tello{} attempting move forward...".format(self.id))
        
        self.y0 = self.y 

        while abs((self.y - self.y0)) < pos[0]:
            print(self.y0, self.y)
            self.vel_pub.publish(twist)
            time.sleep(0.01)

        print(pos[0])
        twist.linear.y = 0


        self.vel = twist
        # Publish
        # rospy.loginfo(str(twist))
        print(twist.linear.x)
        for _ in range(2) :
            self.vel_pub.publish(twist)
            time.sleep(0.2)

    def land(self) :
        '''
            Publishes a land message
            two times in 0.2s intervals.

            After 3s turns off the motors 
            (just in case).
        '''
        msg = Empty()
        for _ in range(2) :
            self.land_pub.publish(msg)
            time.sleep(0.2)
        # self.emergency_pub.publish(msg)

    def positioning(self) :
        '''
            External positioning subscriber
        '''

        self.pos = np.array([self.x, self.y, self.z])
        rospy.loginfo("Tello{} attempting positioning...".format(self.id))
        # if self.connection and self.obj.all() and self.pos.all() :
        if self.connection :
            self.set_speed()


if __name__ == '__main__':
    raise Exception("Do not run this directly!")