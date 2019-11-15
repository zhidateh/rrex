#!/usr/bin/env python

"""
Python MQTT Subscription client - No Username/Password
Thomas Varnish (https://github.com/tvarnish), (https://www.instructables.com/member/Tango172)
Written for my Instructable - "How to use MQTT with the Raspberry Pi and ESP8266"
"""

import time
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import Float32 
import sys

# These functions handle what happens when the MQTT client connects
# to the broker, and what happens then the topic receives a message
def on_connect(client, userdata, flags, rc):
    # rc is the error code returned when connecting to the broker
    print "Connected to MQTT Server!", str(rc)
    print "Publishing to /ldr_node topic"
    # Once the client has connected to the broker, subscribe to the topic
    client.subscribe(mqtt_topic)


    
def on_message(client, userdata, msg):
    # This function is called everytime the topic is published to.
    # If you want to check each message, and do something depending on
    # the content, the code to do this should be run in this function
    #print "Topic: ", msg.topic + "\nMessage: " + str(msg.payload)
    #print float(msg.payload)
    pub.publish(float(msg.payload))
    rospy.loginfo("LDR Value :\t\t%.5f" % float(msg.payload))
    # The message itself is stored in the msg variable
    # and details about who sent it are stored in userdata



if __name__ == '__main__':
    
    pub = rospy.Publisher('/ldr_node/ldr',Float32, queue_size = 10)
    rospy.init_node('ldr_node',anonymous = True)
    print " sda "
    rate = rospy.Rate(20)
    print " sda "

    mqtt_topic = "MQTT_LDR"
    #mqtt_broker_ip = "192.168.0.152"
    
    mqtt_broker_ip = "192.168.4.1"

    client = mqtt.Client()
    client.connect(mqtt_broker_ip, 1883)
        
    # Here, we are telling the client which functions are to be run
    # on connecting, and on receiving a message
    client.on_connect = on_connect
    client.on_message = on_message

    
    while not rospy.is_shutdown():
        # Don't forget to change the variables for the MQTT broker!

        # Once everything has been set up, we can (finally) connect to the broker
        # 1883 is the listener port that the MQTT broker is using
        # Once we have told the client to connect, let the client object run itself
        client.loop(1/60.0)
        #rate.sleep()
        #client.disconnect()
        
    #client.loop_stop()
    client.disconnect()    
    print "\nStopping mqtt... " 
    rospy.spin()
