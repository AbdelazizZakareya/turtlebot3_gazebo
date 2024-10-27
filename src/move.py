#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time  # Import the time module to use wall time

def move():
    # Initialize the ROS node
    rospy.init_node('move_node', anonymous=True)

    # Set up the publisher to publish Twist messages to /cmd_vel
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Set the rate at which to send messages
    rate = rospy.Rate(10)  # 10 Hz

    # Define the Twist message to move forward
    move_cmd = Twist()
    move_cmd.linear.x = 0.2  # Set a forward speed (0.2 m/s)

    # Get the distance from the parameter passed in the launch file
    distance_to_move = rospy.get_param('/move_node/distance', 1.0)  # Default is 1 meter
    print(f"Distnace in .py file {distance_to_move}")

    # Initialize the start time and reset current_distance (using wall time)
    start_time = time.time()  # Use wall time
    current_distance = 0

    # Move the TurtleBot until the desired distance is reached
    while current_distance <= distance_to_move and not rospy.is_shutdown():
        # Log the current distance
        rospy.loginfo(f"Current Distance: {current_distance} meters")
        # Publish the move command
        velocity_publisher.publish(move_cmd)
        # Calculate the current distance moved (using wall time)
        current_time = time.time()  # Use wall time instead of ROS time
        current_distance = move_cmd.linear.x * (current_time - start_time)  # Speed * time

        rate.sleep()

    # Stop the robot after moving the required distance
    move_cmd.linear.x = 0
    velocity_publisher.publish(move_cmd)

    rospy.loginfo("Reached the target distance")

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass