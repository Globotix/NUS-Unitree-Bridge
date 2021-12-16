import paho.mqtt.client as mqtt
import json #for parsing json

import geometry_msgs.msg, std_msgs.msg
import math

import rospy

class MQTTHandler():
    def __init__(self):
        self.client = mqtt.Client("mqtt_test95") #create new instance

    def initMQTTParams(self, navigation_topic, robot_state_topic):
        self.navigation_topic = navigation_topic
        self.robot_state_topic = robot_state_topic
        

    def initMQTTConnection(self, broker_address, broker_port, mqtt_user="", mqtt_password=""):
        #Set username and password if the user and password fields are not empty
        #IMPORTANT THIS MUST BE SET BEFORE client.connect
        if (mqtt_user != "" and mqtt_password != ""):
            self.client.username_pw_set(mqtt_user, mqtt_password)

        #Connect to broker
        self.client.connect(broker_address, broker_port, 60) #connect to broker

        #Register callbacks
        self.client.on_connect = self.on_connect
        # self.client.on_log = self.on_log
        # self.client.on_publish = self.on_publish
        self.client.on_disconnnect = self.on_disconnect

        self.client.on_message = self.on_message

        #Subscribe to navigation topic and marker topic
        self.client.subscribe([(self.navigation_topic, 0)])

    def initROSInterface(self, goal_pub, empty_pub):
        self.goal_pub = goal_pub
        self.empty_pub = empty_pub

    def startLoop(self):
        print("STARTING LOOP")
        self.client.loop_forever()
        # self.client.loop()

    """
    Callback methods for the MQTT client
    """
    
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0: 
            print("Connected to MQTT broker (RC: %s)" % rc)
            self.client.subscribe([(self.navigation_topic, 0)])
        else:
            print("Connection to MQTT broker failed (RC: %s)" % rc)

    def on_log(self, client, userdata, level, buf):
        print(buf)

    def on_publish(self, client, userdata, mid):
        print("Data published (Mid: %s)" % mid)

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnect")
        print("Disconnected from MQTT broker")

    def on_message(self, client, userdata, msg):
        # print("received msg on topic("+msg.topic+") with msg: "+msg.payload)
        print("Received message!")

        if (msg.topic == self.navigation_topic):
            #Convert to ROS Message and publish to ROS
            print("MQTT: Navigation Command received!")

            msg_dict = json.loads(msg.payload)

            if msg_dict["action"] == "dance":
                print("MQTT: Start DANCING! >.<")

            if msg_dict["action"] == "start_movement":
                print("MQTT: Start movement!")

                goal_msg = self.makeROSGoal(msg.payload)

                self.goal_pub.publish(goal_msg)

            elif msg_dict["action"] == "cancel_movement":
                print("MQTT: Cancel movement!")
                empty_msg = std_msgs.msg.Empty()

                self.empty_pub.publish(empty_msg)

            else:
                print("MQTT: INVALID NAVIGATION ACTION")


    """
    Helper methods
    """
 
    def pubMQTT(self, msg_json, topic):
        """
        Publish a JSON message via MQTT 
        """
        self.client.publish(str(topic), msg_json, 1) #last number is qos (quality of service)

    def euler_to_quaternion(self, yaw, pitch, roll):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def makeROSGoal(self, msg_raw_json):

        #1. Convert from JSON bytes to dictionary
        msg_dict = json.loads(msg_raw_json)

        #2. Extract information from JSON
        action = msg_dict["action"]
        x = float(msg_dict["pos_x"])
        y = float(msg_dict["pos_y"])
        # theta = float(msg_dict["pos_theta"])

        #Convert theta to quaternions
        # [q_x, q_y, q_z, q_w] = self.euler_to_quaternion(theta, 0.0, 0.0)

        #Create ROS Message
        goal_msg = geometry_msgs.msg.PoseStamped()

        if (action == "start_movement"):
            goal_msg.header.frame_id = "map"
            goal_msg.header.stamp = rospy.Time.now()

            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.position.z = 0

            # goal_msg.pose.orientation.x = q_x
            # goal_msg.pose.orientation.y = q_y
            # goal_msg.pose.orientation.z = q_z
            goal_msg.pose.orientation.w = 1.0

        elif (action == "dance"):
            goal_msg.header.frame_id = "dance"


        #Debug print statements
        print("Action: {}".format(action))
        print("Goal Position: {}, {}, {}".format(x, y, 0))
        # print("Goal Orientation: {}, {}, {}, {}".format(q_x, q_y, q_z, q_w))


        return goal_msg