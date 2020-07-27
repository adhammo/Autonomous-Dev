#!/usr/bin/env python

import math
from tf.transformations import euler_from_quaternion
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# A simple vector 3 class
class Vec3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return 'x: %.2f, y: %.2f, z: %.2f' % (self.x, self.y, self.z)


# Stores and updates position and rotation of the robot
class Pose:
    def __init__(self):
        self.pos = Vec3(0.0, 0.0, 0.0)
        self.ori = Vec3(0.0, 0.0, 0.0)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        (self.pos.x, self.pos.y, self.pos.z) = (position.x, position.y, position.z)
        (self.ori.x, self.ori.y, self.ori.z) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
    def __str__(self):
        return 'pos: %s\nori: %s' % (self.pos, self.ori)


# Stores and updates the LIDAR raycasts information
class Scan:
    def __init__(self):
        self.ranges = [0.0] * 720
        self.scan_sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.sub_callback)

    def sub_callback(self, msg):
        self.ranges = msg.ranges
        
    def __str__(self):
        return '-90deg: %.2f, 0deg: %.2f, 90deg: %.2f' % (self.ranges[0], (self.ranges[359] + self.ranges[360]) / 2, self.ranges[719])


# Controls the robot linear and angular velocities
class Command:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.linear(0.0, 0.0, 0.0)
        self.angular(0.0, 0.0, 0.0)
    
    def linear(self, x, y, z):
        self.twist.linear.x = x
        self.twist.linear.y = y
        self.twist.linear.z = z
        self.cmd_pub.publish(self.twist)

    def angular(self, x, y, z):
        self.twist.angular.x = x
        self.twist.angular.y = y
        self.twist.angular.z = z
        self.cmd_pub.publish(self.twist)


# Initialize 'loco' node with 60 control messages in a second
rospy.init_node('loco')
rate = rospy.Rate(60)

# Initialize robot instances
pose = Pose()
scan = Scan()
command = Command()

try:
    while not rospy.is_shutdown():
        x = 0.0
        y = 0.0

        # Accumulate error in orientation and speed based on LIDAR readings
        for index in range(720):
            range1 = min(scan.ranges[719 - index], 30)
            range2 = min(scan.ranges[index], 30)
            x += (range1 - range2) * math.cos(index * (math.radians(180) / 719))
            y += (range1 + range2) * math.sin(index * (math.radians(180) / 719))

        # Average velocities
        x /= 720
        y /= 720
        
        # Limit linear speed
        y = min(y, 3)

        # Apply velocities
        command.linear(0.2 * y, 0.0, 0.0)
        command.angular(0.0, 0.0, 0.4 * x)

        # Sleep the rest of the 1.0/60.0 seconds
        rate.sleep()

except rospy.ROSInterruptException:
    pass
    
# Ignored
rospy.spin()