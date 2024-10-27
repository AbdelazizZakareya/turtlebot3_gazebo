#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time  # Import the time module to use wall time
import math

def rotate():
    # Initialize the ROS node
    rospy.init_node('rotate_node', anonymous=True)

    # Set up the publisher to publish Twist messages to /cmd_vel
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Set the rate at which to send messages
    rate = rospy.Rate(10)  # 10 Hz

    # Define the Twist message for rotation
    rotate_cmd = Twist()
    rotate_cmd.angular.z = 0.2  # Set the angular velocity (rad/s)

    # Calculate the angle to rotate (in radians)
    angle_degree = rospy.get_param('angle', 90.0)   # Default is 90 degrees
    angle_to_rotate = math.radians(angle_degree)  # Convert 120 degrees to radians
    offset = math.radians(0.4)
    rospy.loginfo("Rotating by 90 degrees")

    # Initialize the start time and reset current_angle (using wall time)
    start_time = time.time()
    current_angle = 0

    # Rotate the TurtleBot until the desired angle is reached
    while current_angle <= angle_to_rotate+offset and not rospy.is_shutdown():
        # Log the current angle
        rospy.loginfo(f"Current Angle: {math.degrees(current_angle)} degrees")
        # Publish the rotate command
        velocity_publisher.publish(rotate_cmd)
        # Calculate the current angle rotated (using correct angular velocity)
        current_time = time.time()  # Use wall time instead of simulation time
        current_angle = rotate_cmd.angular.z * (current_time - start_time)  # Angular velocity * time
        
        rate.sleep()

    # Stop the robot after reaching the required angle
    rotate_cmd.angular.z = 0
    velocity_publisher.publish(rotate_cmd)

    rospy.loginfo("Reached the target angle")

if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException:
        pass

