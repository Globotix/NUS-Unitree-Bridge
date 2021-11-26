#!/usr/bin/env python3

import sys, os, signal
import math
import threading

import rospy
import geometry_msgs.msg, std_msgs.msg
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from mqtt_handler import *

##################################################
#DEFAULT MQTT CONSTANTS
##################################################

MQTT_BROKER_ADDRESS = str("0.0.0.0") 
MQTT_BROKER_PORT = 1883
MQTT_USER = "guest"
MQTT_PASSWORD = "guest"

MQTT_NAVIGATION_TOPIC = "nus5gdt/robots/unitree/navigate"
MQTT_ROBOT_STATE_TOPIC = "nus5gdt/robots/unitree/robot_state"


def parseConfig(config_dir):
    """
    Function to parse YAML file to populate self.state_dictionary

    @param config_dir [string]: Config file directory
    """
    with open(config_dir) as f:
        dataMap = yaml.safe_load(f) # use safe_load instead of load

        #For each robot state
        for mode in dataMap:
            node_list = []
            #For each node in robot state
            for node in dataMap[mode]:
                node_list.append(node["name"])

            self.state_dictionary[mode] = node_list

parseConfig("./")

#Assign constants
mqtt_broker_address = MQTT_BROKER_ADDRESS
mqtt_broker_port = MQTT_BROKER_PORT
mqtt_user = MQTT_USER
mqtt_password = MQTT_PASSWORD

mqtt_navigation_topic = MQTT_NAVIGATION_TOPIC
mqtt_robot_state_topic = MQTT_ROBOT_STATE_TOPIC

goal_pub = rospy.Publisher('robot_goal', geometry_msgs.msg.PoseStamped, queue_size=10)
empty_pub = rospy.Publisher('cancel_goal', std_msgs.msg.Empty, queue_size=10)

mqtt_handler = MQTTHandler()
#add ros publishers to mqtt_handler
mqtt_handler.initROSInterface(goal_pub, empty_pub)

def robotPositionCallback(data):

    rospy.loginfo("Robot callback received!")

    #Read from ROS msg
    frame_id = data.header.frame_id
    time_stamp = data.header.stamp.secs
    x = data.pose.position.x
    y = data.pose.position.y

    q_x = data.pose.orientation.x
    q_y = data.pose.orientation.y
    q_z = data.pose.orientation.z
    q_w = data.pose.orientation.w

    #Convert quaternion to yaw (about z axis)
    yaw = math.atan2(2.0 * (q_w * q_z + q_x * q_y), q_w**2 + q_x**2 - q_y**2 - q_z**2)
    
    #Create json message for sending to mqtt
    current_pose = {}
    current_pose["x"] = x
    current_pose["y"] = y
    current_pose["theta"] = yaw

    robot_state_msg_dict = {}
    robot_state_msg_dict["current_pose"] = current_pose
    robot_state_msg_dict["time_stamp"] = time_stamp 

    robot_state_msg_json = json.dumps(robot_state_msg_dict)

    #Publish MQTT message
    mqtt_handler.pubMQTT(robot_state_msg_json, mqtt_robot_state_topic)

    return 


class MQTTThread(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        
    def run(self):
        print("Started MQTT Thread")

        mqtt_handler.initMQTTParams(mqtt_navigation_topic, mqtt_robot_state_topic)

        mqtt_handler.initMQTTConnection(mqtt_broker_address, mqtt_broker_port, mqtt_user=mqtt_user, mqtt_password=mqtt_password)
        mqtt_handler.startLoop()


def main():
    thread1 = MQTTThread(1, "mqtt_thread")
    thread1.start()

    rospy.init_node('mqtt_to_unitree_bridge')

    #ROS publishers
    #TODO: add action goal here
    # https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

    #ROS Subscribers
    rospy.Subscriber("robot_position", geometry_msgs.msg.PoseStamped, robotPositionCallback)
    
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