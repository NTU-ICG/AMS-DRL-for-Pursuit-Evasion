#!/usr/bin/env python2

#from _typeshed import NoneType
from logging import RootLogger
import sys
import time
import rospy
import math
import numpy as np

from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Empty

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from tello_driver.msg import TelloStatus

from tello_controller import *

# the nn libs
import onnx  # 1
import onnxruntime as ort # 2
import torch

SWARM_SPEED_STOP_ERROR = 0.1
SWARM_STATUS_THRESHOLD = 0.05


class TelloSwarm():

    def __init__(self, num, objective_pos, tags, objective_paths = None):
        '''
                Swarm controller
        '''

        assert num > 0
        assert num <= len(objective_pos)

        self.tags = tags
        self.bearing31 = None
        self.bearing32 = None
        self.bearing41 = None
        self.bearing42 = None
        self.bearing43 = None

        self.drones = []
        self.swarm_status=[] # to flag each drone if formation is achieved

        # subscribe the goal's position
        self.goal_pos_topic = "/vrpn_client_node/goal/pose"
        # self.position_sub = rospy.Subscriber(self.pos_topic, PoseStamped, self.cb_positioning)
        self.position_sub = rospy.Subscriber(self.goal_pos_topic, PoseStamped, self.goal_positioning)
        self.goal_x = None
        self.goal_y = None
        self.goal_z = None

        self.goal_position = np.array([None, None, None]) 

        for i in range(num):
            self.drones.append(
                TelloController(i,
                                ros_ns="tello{}".format(i),
                                pos_topic="/vrpn_client_node/{}/pose".format(
                                    self.tags[i]),
                                takeoff_height=1.2,
                                external_z=True
                                )
            )
            # [0], objective_pos[i][1], objective_pos[i][2])
            self.drones[i].set_objective(objective_pos[i])
            self.swarm_status.append(False)

        # Delay to
        time.sleep(2)
        rospy.loginfo("Swarm ready.")


    def goal_positioning(self, msg) :
        '''
            External positioning subscriber
        '''
        # H0 = np.array([[0,0,1,0], [1,0,0,0], [0,1,0,0],[0,0,0,1]])
        # position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        # R = 
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.z
        self.goal_z = msg.pose.position.y

        # quaternion = (
        #     msg.pose.orientation.x,
        #     msg.pose.orientation.y,
        #     msg.pose.orientation.z,
        #     msg.pose.orientation.w)
        
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler [2]
        
        self.goal_position = np.array([self.goal_x, self.goal_y, self.goal_z])


    def parallel_takeoff(self):
        '''
            take off the drones
        '''
        for drone in self.drones:
            drone.takeoff()
    

    def agent_takeoff(self, i):
        '''
            take off the drones
        '''
        self.drones[i].takeoff()

    def agent_circle(self, i):
        '''
            take off the drones
        '''
        self.drones[i].circle()
    
    def get_agent_pos(self, i):
        '''
        get the current position of agent i
        '''
        return self.drones[i].pos

    def get_leader_vel(self):
        return [self.drones[0].vel.linear.x, self.drones[0].vel.linear.y, self.drones[0].vel.linear.z]

    def parallel_takeoff_positioning(self):
        '''
            Command all drones to take off simultanously
            (minimal delay if the number is very large)
        '''

        for drone in self.drones:
            drone.takeoff()

        dist = np.ones(len(self.drones))

        while(max(dist) > SWARM_SPEED_STOP_ERROR):
            i = 0

            for drone in self.drones:
                diff = drone.obj[:2] - drone.pos[:2]
                dist_element = np.linalg.norm(diff)
                dist[i] = dist_element
                i += 1
                if drone.is_connection():
                    drone.set_speed_step(diff)
    
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
    
    def agent_positioning(self, i):
        '''
            Command the drone i to take off simultanously
        '''

        drone = self.drones[i]

        diff = drone.obj - drone.pos

        if drone.is_connection():
            drone.set_speed_step_leader(diff)

    def agent_stop(self, i):
        '''
            Command the drone i to take off simultanously
        '''

        drone = self.drones[i]

        diff = [0, 0, 0]

        if drone.is_connection():
            drone.set_speed_step_leader(diff)
    

    def agent_path_following(self, i):
        drone = self.drones[i]
        diff = drone.obj - drone.pos


    def parallel_takeoff_forward(self, delay=0.23):
        '''
                Command all drones to take off simultanously
                (minimal delay if the number is very large)
        '''
        rospy.loginfo("Taking off sequentially...")
        cmd = [1, 0, 0]
        for drone in self.drones:
            drone.takeoff()
            time.sleep(delay)
            drone.forward(cmd)
            drone.land()

    def sequential_takeoff_positioning(self, delay=0.23):
        '''
                Command drones to take off one by one
        '''
        rospy.loginfo("Taking off sequentially...")
        for drone in self.drones:
            drone.takeoff()
            time.sleep(delay)

        for drone in self.drones:
            drone.positioning()
            time.sleep(delay)

    def parallel_land(self):
        '''
                Command all drones to land simultanously
                (minimal delay if the number is very large)
        '''
        for drone in self.drones:
            drone.land()

    def sequential_land(self, delay=0.23):
        '''
                Command drones to land one by one
        '''
        for drone in self.drones:
            drone.land()
            time.sleep(delay)


    def set_trajectory(self, i, pos_traj):
                    # set the path for tello i
            self.drones[i].set_path(pos_traj)

    def set_speed(self, i, vel_linear):
            # set the speed for tello i
            
            self.drones[i].set_speed_step(vel_linear)
            
    # ORIGINAL
    def run_rlInference(self): 
        '''
                Rospy spin while drones go to
                their objective positions
        '''
        rospy.loginfo("Ready to run!")
        for drone in self.drones:
            drone.running = True

        rate = rospy.Rate(100)  #original = 500

        # index = 0

        # num_point_traj = len(self.drones[0].obj_path)

        ############ DRL inference ##############

        sess = ort.InferenceSession("./runner.onnx")
        obs_0 = sess.get_inputs()[0].name
        sess1 = ort.InferenceSession("./chaser.onnx")
        obs_1 = sess1.get_inputs()[0].name
        sess2 = ort.InferenceSession("./chaser.onnx")
        obs_2 = sess2.get_inputs()[0].name

        file = open('result.txt', 'a')
        #" ".join([str(self.goal_x),str(self.goal_y),str(self.goal_z)])
        file.write("goal pos: " + str(self.goal_position)  + "\n")
        file.write("white pos: " + str(self.drones[0].pos) + "\n")
        file.write("blue1 pos: " + str(self.drones[1].pos) + "\n")
        file.write("blue2 pos: " + str(self.drones[2].pos) + "\n")

        while not rospy.is_shutdown():
            
            dist0g = self.drones[0].pos-self.goal_position
            dist01 = self.drones[0].pos-self.drones[1].pos
            dist02 = self.drones[0].pos-self.drones[2].pos
            dist1g = self.drones[1].pos-self.goal_position
            dist10 = self.drones[1].pos-self.drones[0].pos
            dist12 = self.drones[1].pos-self.drones[2].pos
            distabs10 = np.linalg.norm(self.drones[1].pos-self.drones[0].pos)
            distabsfriend = np.linalg.norm(self.drones[1].pos-self.drones[2].pos)
            dist2g = self.drones[2].pos-self.goal_position
            dist20 = self.drones[2].pos-self.drones[0].pos
            dist21 = self.drones[2].pos-self.drones[1].pos
            distabs20 = np.linalg.norm(self.drones[2].pos-self.drones[0].pos)
            distmin0 = np.linalg.norm(dist0g)  
            xposmax = max ([self.drones[0].pos[0], self.drones[1].pos[0], self.drones[2].pos[0]])
            xposmin = min ([self.drones[0].pos[0], self.drones[1].pos[0], self.drones[2].pos[0]])
    
            if(distmin0 > 0.02 and distabs10 > 0.02 and distabs20 > 0.02 and distabsfriend > 0.02):
             
                obs_0_array = np.concatenate((dist0g, dist01, dist02), axis=None)
                obs_0_init= np.array(torch.tensor(np.reshape(obs_0_array, [1,-1]), dtype=torch.float32))
                res = sess.run(None, {obs_0: obs_0_init})
                action = res[2][0]
                self.swarmcontrol_dronetarget(0, action[1], action[2], action[0])

                obs_1_array = np.concatenate((dist10, dist12, distabsfriend), axis=None)
                obs_1_init= np.array(torch.tensor(np.reshape(obs_1_array, [1,-1]), dtype=torch.float32))
                res = sess1.run(None, {obs_1: obs_1_init})
                action = res[2][0]
                self.swarmcontrol_dronechaser1(1, action[1], action[2], action[0])

                obs_2_array = np.concatenate((dist20, dist21, distabsfriend), axis=None)
                obs_2_init= np.array(torch.tensor(np.reshape(obs_2_array, [1,-1]), dtype=torch.float32))
                res = sess2.run(None, {obs_2: obs_2_init})
                action = res[2][0]
                self.swarmcontrol_dronechaser2(2, action[1], action[2], action[0])

                friend = 0
                end = 5
                if (distmin0 < 0.35):
                    self.swarmcontrol_dronetarget(0, 0.0, 0.0, 0.0) # if the distance becomes too small just hovering
                    self.swarmcontrol_dronechaser1(1, 0.0, 0.0, 0.0)
                    self.swarmcontrol_dronechaser2(2, 0.0, 0.0, 0.0)
                    end = 1
                    break

                if (distabsfriend < 0.20):
                    self.swarmcontrol_dronechaser1(1, 0.0, 0.0, 0.0)
                    self.swarmcontrol_dronechaser2(2, 0.0, 0.0, 0.0)
                    friend = 1
                    
                if (distabs10 < 0.35):
                    self.swarmcontrol_dronetarget(0, 0.0, 0.0, 0.0) # if the distance becomes too small just hovering
                    self.swarmcontrol_dronechaser1(1, 0.0, 0.0, 0.0)
                    self.swarmcontrol_dronechaser2(2, 0.0, 0.0, 0.0)
                    end = 2  
                    break

                if (distabs20 < 0.35):
                    self.swarmcontrol_dronetarget(0, 0.0, 0.0, 0.0) # if the distance becomes too small just hovering
                    self.swarmcontrol_dronechaser1(1, 0.0, 0.0, 0.0)
                    self.swarmcontrol_dronechaser2(2, 0.0, 0.0, 0.0)
                    end = 3                 
                    break

                batteries = ""
                for drone in self.drones:
                    batteries += " -- " + str(drone.battery)
                sys.stdout.write('\r Swarm battery levels: {}'.format(batteries))

            rate.sleep()
        
        if friend == 1:
            print("The chaser1 and chaser2 crashed!\n")
            file.write("NA - blues crashed" + "\n")
        if end == 1:
            print("The target arrived!\n")
            file.write("1 - arrived at box" + "\n")
        if end == 2:
            print("The chaser1 caught target!\n")
            file.write("0 - caught by blue1" + "\n")
        if end == 3:
            print("The chaser2 caught target!\n")
            file.write("0 - caught by blue2" + "\n")   

        # rospy.spin()
        self.running = False
        file.close()

    def swarmcontrol_dronetarget(self, i, x, y, z):

        drone = self.drones[i]
        if drone.is_connection():
            vel_linear = [x, y, z]
            drone.set_speed(vel_linear)

    def swarmcontrol_dronechaser1(self, i, x, y, z):
        
        drone = self.drones[i]
        if drone.is_connection():
            vel_linear = [x, y, z]
            drone.set_speed(vel_linear)

    def swarmcontrol_dronechaser2(self, i, x, y, z):

        drone = self.drones[i]
        if drone.is_connection():
            vel_linear = [x, y, z]
            drone.set_speed(vel_linear)

if __name__ == '__main__':
    raise Exception("Do not run this directly!")
