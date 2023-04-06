import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

tello_tags = [
    "tello0",
    "tello1",
    "tello2"
]

car_tags = [
    # "rpi2",
    "rpi3",
    "rpi4",
    "rpi5"
]

rospy.init_node('swarm_land_node', anonymous=True)

for tello in tello_tags:
    land_pub = rospy.Publisher('/{}/land'.format(tello), Empty, queue_size=10)
    msg = Empty()
    for _ in range(2):
        land_pub.publish(msg)
        time.sleep(0.2)

for tello in tello_tags:
    land_pub = rospy.Publisher('/{}/land'.format(tello), Empty, queue_size=10)
    msg = Empty()
    for _ in range(2):
        land_pub.publish(msg)
        time.sleep(0.2)


# for car in car_tags:
#     vel_pub = rospy.Publisher('/{}/cmd_vel'.format(car), Twist, queue_size=10)
    
#     # Message template
#     twist = Twist()
#     twist.linear.x = 0
#     twist.linear.y = 0
#     twist.linear.z = 0

#     # Ensure no rotation
#     twist.angular.x = 0
#     twist.angular.y = 0
#     twist.angular.z = 0
#     for _ in range(2):
#         vel_pub.publish(twist)
#         time.sleep(0.2)