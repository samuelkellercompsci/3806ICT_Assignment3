#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import numpy as np

class SensorNode:
    def __init__(self):
        rospy.init_node('sensor_node')
        self.sensor_pub = rospy.Publisher('sensor_data', String, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def laser_callback(self, data):
        ranges = np.array(data.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) == 0:
            min_range = max_range = average_range = float('inf')
        else:
            min_range = float(np.min(valid_ranges))
            max_range = float(np.max(valid_ranges))
            average_range = float(np.mean(valid_ranges))

        sensor_data = {
            'ranges': valid_ranges.tolist(),
            'min_range': min_range,
            'max_range': max_range,
            'average_range': average_range
        }

        sensor_data_str = json.dumps(sensor_data)
        rospy.loginfo(f"Publishing sensor data: {sensor_data_str}")
        self.sensor_pub.publish(sensor_data_str)

if __name__ == '__main__':
    try:
        SensorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
