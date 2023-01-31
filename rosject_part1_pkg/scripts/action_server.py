#! /usr/bin/env python
import rospy
import math
import numpy as np
import actionlib
from nav_msgs.msg import Odometry
import time

from rosject_part1_pkg.msg import OdomRecordFeedback, OdomRecordResult, OdomRecordAction
from geometry_msgs.msg import Point32, Pose


class OdomClass(object):

    # create messages that are used to publish feedback/result
    _feedback = OdomRecordFeedback()

    def __init__(self):
        # creates the action server
        self._as = actionlib.SimpleActionServer(
            "record_odom", OdomRecordAction, self.goal_callback, False)
        self.sub = rospy.Subscriber(
            '/odom', Odometry, self.sub_cb, queue_size=1)
        self._result = OdomRecordResult()
        self.pose = Point32()
        self._as.start()
        self.ctrl_c = False
        self.rospy = rospy

    def sub_cb(self, msg):
        self.pose = msg.pose.pose.position
        return self.pose

    def distance(self, x_start, y_start, x_current, y_current):
        return np.sqrt((x_current-x_start)**2 + (y_current-y_start)**2)

    def goal_callback(self, goal):
        # this callback is called when the action server is called.
        # this is the function that computes the Fibonacci sequence
        # and returns the sequence to the node that called the action server

        # helper variables
        r = rospy.Rate(1)
        print('action server called')
        success = True

        self.x_start = self.pose.x
        self.y_start = self.pose.y
        self.x_begin = self.pose.x
        self.y_begin = self.pose.y
        self.travelled = 0

        while abs(self.x_begin - self.pose.x) > 0.20 or abs(self.y_begin - self.pose.y) > 0.20 or self.travelled < 1.0:
            # check that preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                # the following line, sets the client in preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the calculation of the Fibonacci sequence
                break

            # legger til current pose i result-listen
            self._result.list_of_odoms.append(self.pose)
            self.rospy.loginfo('x = ' + str(self.pose.x) +
                               '  y = ' + str(self.pose.y))

            self.travelled += self.distance(self.x_start,
                                            self.y_start, self.pose.x, self.pose.y)
            self._feedback.current_total = self.travelled

            self._as.publish_feedback(self._feedback)

            self.x_start = self.pose.x
            self.y_start = self.pose.y
            time.sleep(1)

        if success:
            rospy.loginfo('succeeded')
            self._as.set_succeeded(self._result)


        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If success, then we publish the final result
        # If not success, we do not publish anything in the result
if __name__ == '__main__':
    rospy.init_node('record_odom')
    OdomClass()
    rospy.spin()
