#!/bin/bash

#var for session name
sn=remote_management_node

#start session
tmux new-session -s "$sn" -n etc -d 

tmux split-window -dh $TMUX_PANE
tmux split-window -v $TMUX_PANE
tmux split-window -v -t 0.2 $TMUX_PANE
tmux split-window -v -t 0.3 $TMUX_PANE

#Launch main node
tmux send-keys -t 0.0 "roscore" C-m

sleep 5

#MQTT TEST 1: MQTT Subscription to check robot status
tmux send-keys -t 0.1 "mosquitto_sub -v -h localhost -t \# -u guest -P guest -d" C-m

tmux send-keys -t 0.2 "python3 main.py" C-m

#MQTT TEST 2: MQTT publishing to check sending robot task
# mosquitto_pub -h localhost -t "nus5gdt/robots/unitree/navigate" -u guest -P guest -m "{\"action\":\"start_movement\", \"pos_x\":1.0 , \"pos_y\":2.0 , \"pos_theta\":3.0 }"
# mosquitto_pub -h localhost -t "nus5gdt/robots/unitree/navigate" -u guest -P guest -m "{\"action\":\"cancel_movement\"}"

#ROS Test 1: Send robot position
tmux send-keys -t 0.3 "rostopic pub /robot_position geometry_msgs/PoseStamped " 

#ROS Test 2: Listen to navigation topics
tmux send-keys -t 0.4 "rostopic echo /robot_goal" 


tmux -2 attach-session -d
