# NUS Unitree Intermediary Node

Repo URL: https://github.com/Globotix/NUS-Unitree-Bridge.git

The objective of this node is to act as an intermediary layer between the Unitree API and NUS Digital Twin MQTT interface.
This node should convert MQTT messages from the NUS Digital Twin model to ROS messages for the Unitree API and vice versa.

# Dependencies to be installed

RabbitMQ: Refer to https://www.rabbitmq.com/download.html

#Install the following python packages
```
# paho-mqtt
pip install paho-mqtt

# Mosquitto - MQTT Broker
snap install mosquitto
sudo apt-get install mosquitto
sudo apt-get install mosquitto-clients

# asyncio
pip install asyncio

#Install python installer to package python programs as standalone executables
pip install pyinstaller
```

# Quick Start (ROS)

```
roslaunch --wait unitree_to_nus_bridge unitree_to_nus_bridge.launch
```

# Quick Start


Compile python executable for distribution (Only on the same CPU architecture (ARM/ x86))
```
pyinstaller --onefile main.py
```

Compile Python bytecode .pyc
```
python3 -m compileall -b .
```

# Broker Guide
## Mosquitto

- Start and stop service
```
sudo service mosquitto stop
sudo service mosquitto start #see note later
```

- The stop/start scripts start the mosquitto broker in the background and also use the default mosquitto.conf file in the /etc/mosquitto/ folder.

- Subscribe to all topics
```
mosquitto_sub -v -h localhost -t \# -u guest -P guest -d
```

- Publish on topic "mp/navigate"
```
mosquitto_pub -h localhost -t mp/navigate -u guest -P guest -m {\"hey\":\"HO\"} 

mosquitto_pub -h localhost -t "nus5gdt/robots/unitree/navigate" -u guest -P guest -m "{\"action\":\"start_movement\", \"pos_x\":1.0 , \"pos_y\":2.0 , \"pos_theta\":3.0 }"

mosquitto_pub -h localhost -t "nus5gdt/robots/unitree/navigate" -u guest -P guest -m "{\"action\":\"cancel_movement\"}"

```

# Network settings for connecting to Unitree robot

1. To ~/.bashrc, add the following
```
export ROS_MASTER_URI=http://nx:11311
export ROS_HOSTNAME=192.168.8.149
```

2. To /etc/hosts, add the following
```
192.168.8.204 nx
127.0.0.1 nx
```



