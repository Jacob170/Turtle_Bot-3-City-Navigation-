#! /usr/bin/env python
import rospy
import time
import actionlib

from rosject_part1_pkg.msg import OdomRecordFeedback, OdomRecordResult, OdomRecordAction



# initializes the action client node
rospy.init_node('action_client_node')

# create the connection to the action server
client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
# waits until the action server is up and running
client.wait_for_server()

# creates a goal to send to the action server



# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal("")

# Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()  # would cancel the goal 3 seconds after starting

# wait until the result is obtained
# you can do other stuff here instead of waiting
# and check for status from time to time 
status = client.get_state()
# check the client API link below for more info

client.wait_for_result()

print('[Result] State: %d'%(client.get_state()))