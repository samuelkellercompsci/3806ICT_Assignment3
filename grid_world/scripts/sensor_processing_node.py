#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json

class SensorProcessingNode:
    def __init__(self):
        rospy.init_node('sensor_processing_node')
        self.sensor_sub = rospy.Subscriber('sensor_data', String, self.process_sensor_data)
        self.state_pub = rospy.Publisher('agent_state', String, queue_size=10)

    def process_sensor_data(self, data):
        sensor_data = json.loads(data.data)
        agent_state = {
            'min_range': sensor_data['min_range'],
            'max_range': sensor_data['max_range'],
            'average_range': sensor_data['average_range']
        }
        agent_state_str = json.dumps(agent_state)
        rospy.loginfo(f"Publishing agent state: {agent_state_str}")
        self.state_pub.publish(agent_state_str)

if __name__ == '__main__':
    try:
        SensorProcessingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
