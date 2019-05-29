#!/usr/bin/env python
import serial
import rospy
import time
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Range
import re

############## Methods #############
def RepresentsFloat(s):
	try:
		float(s)
		return True
	except ValueError:
		return False
####################################

# To which topic on Willy we will publish
pubTopicName = 'sonar_data'
subTopicName = 'sonar_command'

# Init ROS components
rospy.init_node('sonar')
pubTopicInstance = rospy.Publisher(pubTopicName, Range ,queue_size=25)

# Init default values
topicMessage = tuple()
TopicDistance = tuple()

# Init serial components
socket = serial.Serial()
socket.baudrate = 9600
socket.port = '/dev/ttyACM0'
socket.timeout = 1
socket.open()

# Function for publish on topic
def PostOnTopic(frameid, Distance):
	message = Range()
	if(RepresentsFloat(Distance)):
		message.header.stamp.secs = rospy.get_rostime().secs
		message.header.stamp.nsecs = rospy.get_rostime().nsecs
		message.header.frame_id = frameid
		message.radiation_type = 0
		message.field_of_view = 1
		message.max_range = 2.0
		message.min_range = 0.1
		message.range = float(Distance) / 100
		pubTopicInstance.publish(message)
		print("message sent on topic: " + frameid + ", value: " + str(message.range))
		#print(Distance)

# Continous loop for publishing serial data
while not rospy.is_shutdown():
	topicMessage = socket.readline()
	topicMessage = topicMessage.rstrip()
	print("--- \n Sonar outputs:")
	print(topicMessage)

	if(re.search(r"\d+\|\d+\|\d+\|\d+\|\d+\|\d+\|\d+\|\d+", topicMessage)):
		PostOnTopic("/front_right",float(topicMessage.split("|")[0]))
		PostOnTopic("/front_middle",float(topicMessage.split("|")[1]))
		PostOnTopic("/front_left",float(topicMessage.split("|")[2]))
		PostOnTopic("/side_left",float(topicMessage.split("|")[3]))
		PostOnTopic("/back_left",float(topicMessage.split("|")[4]))
		PostOnTopic("/back_middle",float(topicMessage.split("|")[5]))
		PostOnTopic("/back_right",float(topicMessage.split("|")[6]))
		PostOnTopic("/side_right",float(topicMessage.split("|")[7]))
