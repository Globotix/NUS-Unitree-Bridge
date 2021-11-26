# NUS Unitree Intermediary Node

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
```

