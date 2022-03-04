#!/usr/bin/env python3

import sys, os, signal
import math
import threading

import yaml #to parse yaml files

import rospy, rosgraph
import tf
import geometry_msgs.msg, std_msgs.msg
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from mqtt_handler import *

##################################################
#DEFAULT MQTT CONSTANTS
##################################################

mqtt_broker_address = ["0.0.0.0"] 
mqtt_broker_port = [1883] 
mqtt_user = ["guest"] 
mqtt_password = ["guest"] 

mqtt_navigation_topic = ["nus5gdt/robots/unitree/navigate"] 
mqtt_robot_state_topic = ["nus5gdt/robots/unitree/robot_state"] 

ros_start_navigation_topic = ["robot_goal"]
ros_cancel_navigation_topic = ["cancel_goal"]
ros_position_topic = ["robot_position"]

def parseConfig(config_dir):
    """
    Function to parse YAML file to populate self.state_dictionary

    @param config_dir [string]: Config file directory
    """
    with open(config_dir) as f:
        dataMap = yaml.safe_load(f) # use safe_load instead of load

        mqtt_broker_address[0] = dataMap["mqtt_broker_address"]
        mqtt_broker_port[0] = dataMap["mqtt_broker_port"]
        mqtt_user[0] = dataMap["mqtt_user"]
        mqtt_password[0] = dataMap["mqtt_password"]

        mqtt_navigation_topic[0] = dataMap["mqtt_navigation_topic"]
        mqtt_robot_state_topic[0] = dataMap["mqtt_robot_state_topic"]

        ros_start_navigation_topic[0] = dataMap["ros_start_navigation_topic"]
        ros_cancel_navigation_topic[0] = dataMap["ros_cancel_navigation_topic"]
        ros_position_topic[0] = dataMap["ros_position_topic"]

parseConfig(os.path.expanduser('~') + "/catkin_ws/src/NUS-Unitree-Bridge/config.yaml")
# parseConfig(os.path.expanduser('~') + "/code_bank/NUS-Unitree-Bridge/config.yaml")

#ROS Publishers
goal_pub = rospy.Publisher(ros_start_navigation_topic[0], geometry_msgs.msg.PoseStamped, queue_size=10)
cancel_goal_pub = rospy.Publisher(ros_cancel_navigation_topic[0], std_msgs.msg.Empty, queue_size=10)

mqtt_handler = MQTTHandler()
#add ros publishers to mqtt_handler
mqtt_handler.initROSInterface(goal_pub, cancel_goal_pub)

def robotPositionCallback(data):

    # rospy.loginfo_throttle(1.0, "Robot callback received!")

    #Read from ROS msg
    frame_id = data.header.frame_id
    time_stamp = data.header.stamp.secs
    x = data.pose.position.x
    y = data.pose.position.y

    q_x = data.pose.orientation.x
    q_y = data.pose.orientation.y
    q_z = data.pose.orientation.z
    q_w = data.pose.orientation.w
    quaternion = [
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w]

    #Convert quaternion to yaw (about z axis)
    # yaw = math.atan2(2.0 * (q_w * q_z + q_x * q_y), q_w**2 + q_x**2 - q_y**2 - q_z**2)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    rospy.loginfo_throttle(1.0, "Position xy(%f, %f), rpy_rad(%f, %f %f)", x, y, roll, pitch, yaw)
    rospy.loginfo_throttle(1.0, "rpy_deg(%f, %f %f)", roll*57.2958, pitch*57.2958, yaw*57.2958)
    rospy.loginfo_throttle(1.0, "Quaternion %f, %f, %f, %f", q_x, q_y, q_z, q_w)

    #Create json message for sending to mqtt
    current_pose = {}
    current_pose["x"] = str(round(x, 2))
    current_pose["y"] = str(round(y, 2))
    current_pose["theta"] = str(round(yaw, 2))

    robot_state_msg_dict = {}
    robot_state_msg_dict["current_pose"] = current_pose
    robot_state_msg_dict["time_stamp"] = time_stamp 

    robot_state_msg_json = json.dumps(robot_state_msg_dict)

    #Publish MQTT message
    mqtt_handler.pubMQTT(robot_state_msg_json, mqtt_robot_state_topic[0])

    return 


class MQTTThread(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        
    def run(self):
        print("Started MQTT Thread")

        mqtt_handler.initMQTTParams(mqtt_navigation_topic[0], mqtt_robot_state_topic[0])

        mqtt_handler.initMQTTConnection(mqtt_broker_address[0], mqtt_broker_port[0], mqtt_user=mqtt_user[0], mqtt_password=mqtt_password[0])
        mqtt_handler.startLoop()


def main():
    thread1 = MQTTThread(1, "mqtt_thread")
    thread1.start()

    # ros_master_online = False
    # while not ros_master_online:
    #     try:
    #         if rosgraph.is_master_online():
    #             print("ROS Master is online")
    #             ros_master_online = True
    #         else:
    #             print("ROS Master is not up")
    #     except KeyboardInterrupt:
    #         sys.exit(0)

    rospy.init_node('mqtt_to_unitree_bridge')

    #ROS Subscribers
    rospy.Subscriber(ros_position_topic[0], geometry_msgs.msg.PoseStamped, robotPositionCallback)
    
    while (not rospy.is_shutdown()):
        rospy.spin()


def keyboard_interrupt_handler(signal, frame):
    """
    Handles signal interruption
    """
    print("Keyboard interrupt detected. Exiting gracefully...")
    sys.exit(0)

if __name__ == "__main__":
    #Handle keyboard interrupts
    signal.signal(signal.SIGINT, keyboard_interrupt_handler)

    main()