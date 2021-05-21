#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import paho.mqtt.client as mqtt

broker_url = "broker.mqttdashboard.com"
broker_port = 1883

pub = rospy.Publisher('joint_angles', String, queue_size=10)

def on_connect(client, userdata, flags, rc):
    print("[INFO] Connected With Result Code: " + str(rc))

def on_message(client, userdata, message):
    print("--- Subscriber ---")
    print("[INFO] Topic: {}".format(message.topic) )
    print("[INFO] Message Recieved: {}".format(message.payload.decode()))
    pub.publish(message.payload.decode())
    print("------------")


def talker():
    
    rospy.init_node('get_iot_data', anonymous=True)
    while not rospy.is_shutdown():
        sub_client = mqtt.Client()
        sub_client.on_connect = on_connect
        sub_client.on_message = on_message
        sub_client.connect(broker_url, broker_port)
        sub_client.subscribe("/iot/arm_joint_angles", qos=0)
        sub_client.loop_forever()
    #rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass