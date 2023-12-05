#!/usr/bin/env python3
import rospy
import actionlib
from action_setup.msg import MissionActionAction, MissionActionResult, MissionActionFeedback
from std_msgs.msg import Float32, Int64MultiArray, Bool

class MissionActionServer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('mission_server_node')
        # Create an action server
        self.server = actionlib.SimpleActionServer('mission_server', MissionActionAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        # Process the goal and send a result back to the client
        result = MissionActionResult()
        feedback = MissionActionFeedback()
        result.result = "Goal successfully received and executed"

        # You can perform your desired action logic here using the goal data
        rospy.loginfo('Received goal: {}'.format(goal))

        # perform the desired action to achieve goal state 
        # using the yaw data turn the rover to that angle, during turning the rover that must reflect in the client side also.
        # after turning if obstacle info is false, then start cmd vel and move the rover towards the arrow mark.
        # if the obstacle is in close proximity stop the rover.
        # In between we can give feedback using self.server.publish_feedback(feedback)
        rospy.loginfo('Feedback will be provided according to work done.')
        
        # Send the result back to the client
        self.server.set_succeeded(result)

if __name__ == '__main__':
    mission_server = MissionActionServer()
    rospy.spin()
