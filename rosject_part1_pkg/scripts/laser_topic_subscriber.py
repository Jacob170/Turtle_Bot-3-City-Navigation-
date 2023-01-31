#! /usr/bin/env python
# this is subscriber and publisher on the same Node!
# I subscribed to the Laser data and publish New Velocities under the Twist() topic!
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rosject_part2_pkg.srv import FindWall, FindWallRequest


Need_calibration = True


# Initiate a Node called 'topic_subscriber'
rospy.init_node('laser_subscriber_node')


def service_call_func():

    # Wait for the service client /trajectory_by_name to be running
    rospy.wait_for_service('/find_wall')
    # Create the connection to the service
    find_wall_service = rospy.ServiceProxy('/find_wall', FindWall)
    # Create an object of type TrajByNameRequest
    find_wall_object = FindWallRequest()
    # Fill the variable traj_name of this object with the desired value
    find_wall_object.str = 'go'
    result = find_wall_service(find_wall_object)
    print("result", result)
    if result is True:
        Need_calibration = False


if Need_calibration is True:
    service_call_func()

first_run = True
last_range = 0.0


def shutdownhook():
    # define the above mentioned function and send the twist messages here

    vel.linear.x = 0.0
    vel.angular.z = 0.0
    pub.publish(vel)


# Define a function called 'callback' that receives a parameter
def callback(msg):
    global vel
    global first_run
    global last_range
    vel = Twist()
    vel.linear.x = 0.0
    print("\n")
    print("last range =",last_range)
    print("\n")
    avg_frnt = (msg.ranges[359]+msg.ranges[360]+msg.ranges[361])/3
    rospy.loginfo(rospy.get_name() + ': Current avg_frnt {0}'.format(avg_frnt))

    rospy.loginfo(rospy.get_name() +
                  ': Current 179 element {0}'.format(msg.ranges[179]))

    #if first_run:

    if msg.ranges[179] <= 0.31 and msg.ranges[179] > 0.29:
        print("between 0.32 - 0.28")
        vel.angular.z = 0.0
        vel.linear.x = 0.1

    elif msg.ranges[179] <= 0.29 and msg.ranges[179] > 0.27:
        print("range > 27 and range <=29")
        vel.angular.z = 0.12
        vel.linear.x = 0.08

    elif msg.ranges[179] > 0.31 and msg.ranges[179] <= 0.33:
        print("range > 31 and range <=33")
        vel.angular.z = -0.12
        vel.linear.x = 0.08

    elif msg.ranges[179] > 0.33:
        print("beam > 0.33")
        vel.angular.z = -0.1
        vel.linear.x = 0.00
        if last_range > msg.ranges[179]:
            vel.angular.z +=0.01
    else:
        print(" beam < 0.27")
        vel.angular.z = 0.1
        vel.linear.x = 0.00
        if last_range < msg.ranges[179]:
            vel.angular.z -=0.01

    if avg_frnt < 0.35:
        vel.angular.z = 0.30
        vel.linear.x = 0.02

    rate.sleep()
    last_range = msg.ranges[179]    
        
    """
        if avg_frnt <= 0.25:
            vel.angular.z = 0.0
            vel.linear.x = -0.15
        last_range = msg.ranges[179]
        first_run = False
    else:

        if last_range <= msg.ranges[179] and msg.ranges[179] >= 0.30:
            vel.angular.z = -0.03
            vel.linear.x = 0.07
        elif last_range < msg.ranges[179] and msg.ranges[179] < 0.29:
            vel.angular.z = 0.00
            vel.linear.x = 0.09
        elif last_range >= msg.ranges[179] and msg.ranges[179] < 0.30:
            vel.angular.z = 0.03
            vel.linear.x = 0.09
        elif last_range > msg.ranges[179] and msg.ranges[179] >= 0.31:
            vel.angular.z = 0.00
            vel.linear.x = 0.09
        if avg_frnt < 0.35:
            vel.angular.z = 0.30
            vel.linear.x = 0.02

        if avg_frnt <= 0.25:
            vel.angular.z = 0.0
            vel.linear.x = -0.10
        if avg_frnt > 0.4 and vel.linear.x < 0:
            vel.linear.x = 0.0
        rate.sleep()
        last_range = msg.ranges[179]
    """
    pub.publish(vel)


sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# rospy.loginfo("the 539 element of laser=", LaserScan.ranges[539])
# Create a Subscriber object that will listen to the /counter
# ray_angle90 = LaserScan.angle_min + (179 * LaserScan.angle_increment)
# rate = rospy.Rate(2)
print("sub", sub)

rate = rospy.Rate(10)
str_pub = "publishing"

rospy.loginfo(str_pub)

vel = Twist()

rospy.on_shutdown(shutdownhook)  # add this line to your main program


rospy.spin()

# topic and will cal the 'callback' function each time it reads
# something from the topic
# Create a loop that will keep the program in execution
