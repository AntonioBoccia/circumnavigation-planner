#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms
import std_msgs.msg as sms
import threading as thd
import numpy as np
import geometry_msgs.msg as gm


rp.init_node('sensor_simulator')

position = None
distance = 0
TARGET_POSITION = np.array(rp.get_param('target_position')) #from the .yaml file

LOCK = thd.Lock()

#Subscriber
def position_callback(msg):
    global position
    LOCK.acquire()
    position = np.array([msg.x, msg.y])
    LOCK.release()
rp.Subscriber(
    name='position',
    data_class=gms.Point,
    callback=position_callback,
    queue_size=10)

# Rate
rate=rp.get_param('rate')
RATE = rp.Rate(rate)

start = False
#Publisher
bearing_pub = rp.Publisher(
    name='bearing_measurement',
    data_class=gms.Vector,
    queue_size=10)
distance_pub = rp.Publisher(
    name='distance',
    data_class=gm.PointStamped,
    queue_size=10)

while not rp.is_shutdown() and not start:
    LOCK.acquire()
    if not position is None:
        start = True
    #else:
        #rp.logwarn('waiting for position')
    LOCK.release()
    RATE.sleep()
while not rp.is_shutdown():
    LOCK.acquire()
    #Bearing vector (phi)
    bearing = (TARGET_POSITION-position)/np.linalg.norm(TARGET_POSITION-position)
    distance=np.linalg.norm(TARGET_POSITION-position)
    #Bearing vector publishing
    bearing_msg = gms.Vector(*bearing)
    bearing_pub.publish(bearing_msg)
    #Distance publishing
    distance_msg=gm.PointStamped()
    distance_msg.header.seq=0
    distance_msg.header.stamp = rp.Time.now()
    distance_msg.point.x=distance
    distance_msg.point.y=0
    distance_msg.point.z=0
    distance_pub.publish(distance_msg)
    LOCK.release()
    RATE.sleep()
