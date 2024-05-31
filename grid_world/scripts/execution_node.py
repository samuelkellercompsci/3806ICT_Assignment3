#!/usr/bin/env python3

import rospy
from grid_world.srv import Sensor, SensorResponse, UpdateGrid
from geometry_msgs.msg import Twist
import time

# Initialize the ROS node
rospy.init_node('execution_node')

# Service proxies
package_sensor_client = rospy.ServiceProxy('package_sensor', Sensor)
shelf_sensor_client = rospy.ServiceProxy('shelf_sensor', Sensor)
update_grid_client = rospy.ServiceProxy('update_grid', UpdateGrid)

# Publisher for robot movement
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def move_robot(linear_speed, angular_speed):
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    cmd_vel_pub.publish(twist)
    time.sleep(1)  # Move for 1 second
    twist.linear.x = 0
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)

def main():
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        # Call the package sensor service
        package_sensor_response = package_sensor_client(sensorRange=3)
        if package_sensor_response.objectDetected:
            rospy.loginfo("Package detected, updating grid.")
            update_grid_client(package_sensor_response.grid)

        # Call the shelf sensor service
        shelf_sensor_response = shelf_sensor_client(sensorRange=3)
        if shelf_sensor_response.objectDetected:
            rospy.loginfo("Shelf detected, updating grid.")
            update_grid_client(shelf_sensor_response.grid)

        # Simple logic to move the robot
        move_robot(0.2, 0)  # Move forward
        rate.sleep()

if __name__ == '__main__':
    main()
