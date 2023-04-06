#! /usr/bin/python

import sys
import time
import rospy
import roslaunch

'''
   For reference:
      <arg name="namespace"			default="tello" />
      <arg name="tello_ip" 			default="192.168.10.1" />
      <arg name="local_cmd_client_port"	default="8890" />
      <arg name="local_vid_server_port"	default="6038" />
      <arg name="tello_cmd_server_port"	default="8889" />
'''

# class SwarmLauncher() :

#    def __init__(self) :

tello_ip_start = 120
tello_port = 8889

local_port_start = 8890
local_vid_port_start = 6038   # NOT WORKING ATM

'''
   Number of nodes (IPs considered consecutive)
   and list of matching UWB active tags
'''
num_nodes = 3
if len(sys.argv) > 0 :
   try : 
      num_nodes = int(sys.argv[1])
   except :
      print("Cannot read number of drones (sys.argv[1]). Setting num_nodes=1...")

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch_file = "tello_node_multi.launch"
launch_files = []

'''

   Launch Tello node for each

'''
for num in range(num_nodes):

   cli_args = ['tello_driver', 
               launch_file, 
               'namespace:=tello{}'.format(num), 
               'tello_ip:=192.168.0.{}'.format(tello_ip_start + num), 
               'local_cmd_client_port:={}'.format(local_port_start + num),
               'local_vid_server_port:={}'.format(local_vid_port_start + num), 
               'tello_cmd_server_port:={}'.format(tello_port)
            ]
   roslaunch_args = cli_args[2:]
   roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
   print(cli_args, " ===>>>  ", num)
   
   launch_files=[(roslaunch_file, roslaunch_args)]

   parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

   parent.start()


try :
   while not rospy.is_shutdown() :
      time.sleep(0.1)
except :
   print("Exiting...")
   pass

