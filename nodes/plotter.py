#! /usr/bin/python

import rospy as rp
import geomtwo.msg as gms

import threading as thd
import copy as cp

import matplotlib.pyplot as plt

import dynamic_network_estimate.srv as dns

rp.init_node('plotter')



AGENT_COLOR = rp.get_param('agent_color','blue')
ESTIMATE_COLOR = rp.get_param('estimate_color','red')
TARGET_COLOR = rp.get_param('target_color','black')

#AGENT_NAMES = rp.get_param('agent_names').split()
agent_names=[]
TARGET_POSITION = rp.get_param('target_position')

RATE = rp.Rate(3.0e1)

LOCK = thd.Lock()

plt.ion()
plt.figure()
plt.scatter(*TARGET_POSITION, color=TARGET_COLOR)
plt.axis('equal')
plt.grid(True)
plt.draw()

#agent_positions = {name: None for name in AGENT_NAMES}
#agent_artists = {name: None for name in AGENT_NAMES}

#estimates = {name: None for name in AGENT_NAMES}
#estimate_artists = {name: None for name in AGENT_NAMES}
agent_positions={}
agent_artists={}

agent_estimates={}
estimate_artists={}

# Service Handle
def add_agent_artist_handler(req):
    rp.Subscriber(
        name='/'+req.name+'/position',
        data_class=gms.Point,
        callback=agent_callback,
        callback_args=req.name,
        queue_size=1)
    rp.Subscriber(
        name='/'+req.name+'/estimate',
        data_class=gms.Point,
        callback=estimate_callback,
        callback_args=req.name,
        queue_size=1)
    LOCK.acquire()
    agent_names.append(req.name)
    agent_positions[req.name]=None
    agent_estimates[req.name]=None
    agent_artists[req.name]=None
    estimate_artists[req.name]=None
    LOCK.release()
    return dns.AddAgentResponse()   


rp.Service('AddAgentArtist', dns.AddAgent, add_agent_artist_handler)

def agent_callback(msg, name):
    global agent_positions
    LOCK.acquire()
    agent_positions[name] = [msg.x, msg.y]
    LOCK.release()
# for name in agent_names:
#     rp.Subscriber(
#         name=name+'/position',
#         data_class=gms.Point,
#         callback=agent_callback,
#         callback_args=name,
#         queue_size=1)

def estimate_callback(msg, name):
    global agent_estimates
    LOCK.acquire()
    agent_estimates[name] = [msg.x, msg.y]
    LOCK.release()
# for name in agent_names:
#     rp.Subscriber(
#         name=name+'/estimate',
#         data_class=gms.Point,
#         callback=estimate_callback,
#         callback_args=name,
#         queue_size=1)

# def estimate_callback(msg):
#     global estimate
#     LOCK.acquire()
#     estimate = [msg.x, msg.y]
#     LOCK.release()
# rp.Subscriber(name='estimate',
#               data_class=gms.Point,
#               callback=estimate_callback)

while not rp.is_shutdown():
    ag_pos = {}
    est = {}
    LOCK.acquire()
    for name in agent_names:
        if not agent_positions[name] is None:
            ag_pos[name] = cp.copy(agent_positions[name])
            agent_positions[name] = None
        if not agent_estimates[name] is None:
            est[name] = cp.copy(agent_estimates[name])
            agent_estimates[name] = None
    LOCK.release()
    for name,pos in ag_pos.items():
        if not pos is None:
            if not agent_artists[name] is None:
                agent_artists[name].remove()
            agent_artists[name] = plt.scatter(*ag_pos[name], color=AGENT_COLOR)
    for name,estim in est.items():
        if not estim is None:
            if not estimate_artists[name] is None:
                estimate_artists[name].remove()
            estimate_artists[name] = plt.scatter(*est[name], color=ESTIMATE_COLOR)
    plt.draw()
    RATE.sleep()
