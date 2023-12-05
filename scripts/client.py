#!/usr/bin/env python3
import rospy
import actionlib
from action_setup.msg import MissionActionAction, MissionActionGoal
from std_msgs.msg import Float32, Int64MultiArray, Bool
import time

def send_data_to_server(client):
    # Set different data types for testing
    yaw_angle = 45.0
    x_coordinates = [1, 2, 3]
    y_coordinates = [4, 5, 6]
    obstacle_info = True

    # Create a MissionAction goal with the test data
    goal = MissionActionGoal(
        yaw_angle=yaw_angle,
        x_coordinates=x_coordinates,
        y_coordinates=y_coordinates,
        obstacle_info=obstacle_info
    )

    # Log the sent goal information
    rospy.loginfo('\nSending goal:\n{}'.format(goal))


    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the result (you may adjust this based on your requirements)
    client.wait_for_result()

    # Get the result (if needed)
    result = client.get_result()
    if result:
        rospy.loginfo('\nReceived result: \n{}'.format(result.result))

if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('watchdog_client_node', anonymous=True)

        # Create an action client outside the loop
        client = actionlib.SimpleActionClient('mission_server', MissionActionAction)
        if client.wait_for_server():
            rospy.loginfo('Connection to action server established successfully!')
        else:
            rospy.logwarn('Failed to establish connection to action server. Exiting...')
            exit()

        # Set the desired frequency (e.g., every 2 seconds)
        rate = rospy.Rate(0.5)  # 0.5 Hz, which corresponds to every 2 seconds

        while not rospy.is_shutdown():
            # Manually send data to the server for testing
            send_data_to_server(client)

            # Wait for the specified rate
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
