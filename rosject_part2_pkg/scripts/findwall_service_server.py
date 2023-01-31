#! /usr/bin/env python

import rospy

from rosject_part2_pkg.srv import FindWall, FindWallResponse
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# flag for checking if minimum value obtained
minimum_set = False

# flag to check if wall_is found
found_the_wall = False

#minimum = 0

# service call back function


def service_callback(request):
    rospy.loginfo("The Service /findWall has been called")
    global found_the_wall

    ready_to_turn = False

    #min_ray_angle = laser_data.angle_min + (index * laser_data.angle_increment)
    rate.sleep()
    print('minimum: ', minimum)
    print("len ranges", len(laser_data.ranges))
    print('index: ', index)
    if index < 361 and index >= 359 and ready_to_turn is False:
        print("index IS between 359 - 360 ")
        # minimum is in front
        while laser_data.ranges[359] >= 0.3:

            move.linear.x = 0.05
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()
        print("Moved facing the wall!")
        last_range = laser_data.ranges[359]
        ready_to_turn = True
    else:
        print("index not between 359 - 360 ")
        # the minimum is not in front of robot
        #avg = (laser_data.ranges[359] + laser_data.ranges[360] + laser_data.ranges[358])/3
        while (laser_data.ranges[359] - minimum) > 0.05:
            print("minimum = ", minimum)
            print("\n")
            print("laser_data.ranges[359] = ", laser_data.ranges[359])
            print("\n")
            if index < 360:
                print("index < 360 ")
                move.angular.z = -0.15  # turn right
                pub.publish(move)
                rate.sleep()
            else:
                #index > 360
                move.angular.z = 0.15  # turn right
                print("index > 360 ")
                pub.publish(move)
                rate.sleep()

        while laser_data.ranges[359] >= 0.3:
            print(" laser_data.ranges[359] >= 0.3 inside while loop")
            move.linear.x = 0.05
            move.angular.z = 0.0
            pub.publish(move)
            rate.sleep()
        print("Moved facing the wall!")
        last_range = laser_data.ranges[359]
        ready_to_turn = True
        move.linear.x = 0.00
        move.angular.z = 0.0
        pub.publish(move)
    print("ready to turn", ready_to_turn)

    if ready_to_turn:
        if (laser_data.ranges[269] - last_range) <= 0.05:

            print("laser_data.ranges[269] <=  min(laser_data.ranges)")

            move.angular.z = 0  # stop
            move.angular.x = 0
            pub.publish(move)
            rate.sleep()
            print("Done !")
            found_the_wall = True
        else:
            print("laser_data.ranges[269] -  last_range) > 0.01")
            while (laser_data.ranges[269] - last_range) > 0.01:
                print("last_range =", last_range)
                print("laser_data.ranges[269] =", laser_data.ranges[269])
                move.angular.z = -0.15  # turn left
                move.angular.x = 0.0
                pub.publish(move)
                rate.sleep()
            print("Done !")
            found_the_wall = True
            move.linear.x = 0.00
            move.angular.z = 0.0
            pub.publish(move)

    resp = FindWallResponse()

    if found_the_wall is True:

        resp.wallfound = True
        return resp  # the service Response class, in this case FindWall
    else:
        return 0
    rospy.spin()

# scan callback function will run only once to find the wall direction.


def scan_callback(msg):
    global laser_data
    global minimum
    global index
    global minimum_set

    if minimum_set is False:
        print("scan_callback of wall find")

        #lasers = request

        #right_half_array = msg.ranges[0:380:]
        minimum = min(msg.ranges)

        print("minimum is: ", minimum)
        laser_data = msg
        index = msg.ranges.index(minimum)
        minimum_set = True
        return 0


rospy.init_node('findwall_service_server_node')
sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
rate = rospy.Rate(10)
laser_data = LaserScan()
rate.sleep()
rate.sleep()
rate.sleep()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate.sleep()
rate.sleep()
rate.sleep()

# start service only if minimum value already obtained
if minimum_set is True:
    # create the Service called find_wall with the defined callback
    my_service = rospy.Service('/find_wall', FindWall, service_callback)
    rospy.loginfo("The Service is ready to be called")
move = Twist()


rospy.spin()
