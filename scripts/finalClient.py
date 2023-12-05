#!/usr/bin/env python3

import rospy
from action_setup.msg import MissionActionAction, MissionActionGoal
from std_msgs.msg import String, Int64MultiArray, Bool, Float32

class WatchdogClient:
    def __init__(self):
        rospy.init_node('watchdog_client_node', anonymous=True)

        self.yaw_angle = None
        self.x_coordinates = None
        self.y_coordinates = None
        self.obstacle_info = None

        self.subscriber_yaw = rospy.Subscriber('topic_yaw', Float32, self.callback_yaw)
        self.subscriber_x = rospy.Subscriber('topic_x_coordinates', Int64MultiArray, self.callback_x_coordinates)
        self.subscriber_y = rospy.Subscriber('topic_y_coordinates', Int64MultiArray, self.callback_y_coordinates)
        self.subscriber_obstacle = rospy.Subscriber('topic_obstacle_info', Bool, self.callback_obstacle_info)

        self.publisher = rospy.Publisher('server_topic', MissionActionAction, queue_size=10)

    def callback_yaw(self, data):
        self.yaw_angle = data.data
        self.send_data_to_server()

    def callback_x_coordinates(self, data):
        self.x_coordinates = data.data
        self.send_data_to_server()

    def callback_y_coordinates(self, data):
        self.y_coordinates = data.data
        self.send_data_to_server()

    def callback_obstacle_info(self, data):
        self.obstacle_info = data.data
        self.send_data_to_server()

    def send_data_to_server(self):
        if all(data is not None for data in [self.yaw_angle, self.x_coordinates, self.y_coordinates, self.obstacle_info]):
            combined_data = MissionActionGoal(
                yaw_angle=self.yaw_angle,
                x_coordinates=self.x_coordinates,
                y_coordinates=self.y_coordinates,
                obstacle_info=self.obstacle_info
            )
            self.publisher.publish(combined_data)

if __name__ == '__main__':
    watchdog_client = WatchdogClient()
    rospy.spin()