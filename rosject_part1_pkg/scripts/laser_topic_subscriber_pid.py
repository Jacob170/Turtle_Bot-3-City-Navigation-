#! /usr/bin/env python
# this is subscriber and publisher on the same Node!
# I subscribed to the Laser data and publish New Velocities under the Twist() topic!

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rosject_part2_pkg.srv import FindWall, FindWallRequest


class WallFollower:
    def __init__(self):
        # Initialize PID constants
        self.Kp = 0.5
        self.Ki = 0.0
        self.Kd = 0.1
        self.Need_calibration = True
        self.rate = 20  # Hz
        self.r = rospy.Rate(self.rate)
        # Initialize error values
        self.error_sum = 0.0
        self.last_error = 0.0
        # Initialize obstacle avoidance flag
        self.avoid_obstacle = False
        self.stuckon_on_wall = False
        # Set the desired distance from the wall
        self.desired_distance = 0.24
        rospy.on_shutdown(self.shutdownhook)
        # Subscribe to the LIDAR scan data

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Publish to the turtlebot's velocity topic
        self.velocity_publisher = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=10)
        print("sub")
        if self.Need_calibration is True:
            self.service_call_func()

    def service_call_func(self):

        # Wait for the service client /trajectory_by_name to be running
        rospy.wait_for_service('/find_wall')
        # Create the connection to the service
        find_wall_service = rospy.ServiceProxy('/find_wall', FindWall)
        # Create an object of type TrajByNameRequest
        find_wall_object = FindWallRequest()
        # Fill the variable traj_name of this object with the desired value
        find_wall_object.str = 'go'
        result = find_wall_service(find_wall_object)
        print("result of finding wall = ", result)
        if result != 0:
            self.Need_calibration = False
            print("need caliration", self.Need_calibration)

    #first_run = True
    #last_range = 0.0

    def scan_callback(self, data):
        # Get the distance to the wall on the right side
        if self.Need_calibration is False:
            front_distance = (data.ranges[359] +
                              data.ranges[355] + data.ranges[364]) / 3
            print("front_distance = ", front_distance)

            # Calculate the error

            if front_distance < 0.4:
                self.avoid_obstacle = True
            # If avoiding an obstacle, turn right until the obstacle disappears
            if self.avoid_obstacle:
                vel_msg = Twist()
                vel_msg.linear.x = 0.05
                vel_msg.angular.z = 0.35
                self.velocity_publisher.publish(vel_msg)
                if front_distance > 0.5:
                    self.avoid_obstacle = False
                return

            right_distance = data.ranges[179]
            print("right_distance = ", right_distance)

            if right_distance > 100:
                self.stuckon_on_wall = True
                vel_msg = Twist()
                vel_msg.linear.x = 0.05
                vel_msg.angular.z = 0.5
                self.velocity_publisher.publish(vel_msg)
                return

            if self.stuckon_on_wall == True and right_distance < 100:
                self.stuckon_on_wall = False

            error = self.desired_distance - right_distance

            # Calculate the integral
            self.error_sum += error

            # Calculate the derivative
            error_derivative = error - self.last_error

            # Calculate the output
            output = self.Kp * error + self.Ki * self.error_sum + self.Kd * error_derivative

            print("Output = ", output)

            # Create a Twist message to control the turtlebot's velocity
            vel_msg = Twist()
            vel_msg.angular.z = output
            vel_msg.linear.x = 0.04
            # Publish the velocity message
            str_pub = "publishing"

            rospy.loginfo(str_pub)
            self.velocity_publisher.publish(vel_msg)

            # Update the previous error
            self.last_error = error
            self.r.sleep()

    def shutdownhook(self):
        # define the above mentioned function and send the twist messages here
        rospy.loginfo("Stopping the robot...")
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        # rospy.sleep(1)
        return
    """
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
    
        pub.publish(vel)


    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # rospy.loginfo("the 539 element of laser=", LaserScan.ranges[539])
    # Create a Subscriber object that will listen to the /counter
    # ray_angle90 = LaserScan.angle_min + (179 * LaserScan.angle_increment)
    # rate = rospy.Rate(2)

    """


if __name__ == '__main__':
    rospy.init_node('laser_subscriber_node')

    vel = Twist()
    wall_follower = WallFollower()
    # add this line to your main program

    rospy.spin()


# topic and will cal the 'callback' function each time it reads
# something from the topic
# Create a loop that will keep the program in execution
