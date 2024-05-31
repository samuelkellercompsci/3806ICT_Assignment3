#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point
from grid_world.srv import UpdateGrid, UpdateGridResponse, Sensor, SensorResponse
from gazebo_msgs.srv import GetModelState, SpawnModel, DeleteModel, SetModelState
import os
import random
import math
import subprocess

# Constants
GRID_WIDTH = 1.0
BOARD_H = 10  # Example height, replace with actual value
BOARD_W = 10  # Example width, replace with actual value
EMPTY, PACKAGE, SHELF, ROBOT, VISITED = 0, 1, 2, 3, 4
PAT_EXECUTABLE = "~/catkin_ws/pat/PAT3.Console.exe"

# Global variables
currentGrid = [[EMPTY] * BOARD_W for _ in range(BOARD_H)]
coordinates = [[Point() for _ in range(BOARD_W)] for _ in range(BOARD_H)]
objectPositions = {}
numPackages = 0
numShelves = 0
robotSpawned = False

# Initialize the ROS node
rospy.init_node('ai_interaction_node')
bot_location = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
spawn_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
delete_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

def call_pat(csp_file):
    """ Call the PAT executable with a given .csp file and return the output """
    result = subprocess.run([PAT_EXECUTABLE, csp_file], capture_output=True, text=True)
    return result.stdout

# Initialize coordinates
for i in range(BOARD_H):
    for j in range(BOARD_W):
        coordinates[i][j].x = i * GRID_WIDTH
        coordinates[i][j].y = j * GRID_WIDTH
        coordinates[i][j].z = 0

def create_spawn_request(model_type, position):
    global numPackages, numShelves
    spawn = SpawnModel()
    model_dir = os.path.join(os.getenv("HOME"), "catkin_ws/src/grid_world/models/")
    if model_type == PACKAGE:
        spawn.model_name = "package" + str(numPackages)
        numPackages += 1
        model_path = os.path.join(model_dir, "cardboard_box/model.sdf")
    elif model_type == SHELF:
        spawn.model_name = "shelf" + str(numShelves)
        numShelves += 1
        model_path = os.path.join(model_dir, "bowl/model.sdf")
    elif model_type == ROBOT:
        spawn.model_name = "sorting_robot"
        model_path = os.path.join(model_dir, "turtlebot3_burger/model.sdf")

    with open(model_path, 'r') as file:
        spawn.model_xml = file.read()

    spawn.initial_pose.position = position
    return spawn

def update_grid(req):
    global currentGrid, robotSpawned
    read_grid = req.grid
    for i in range(BOARD_H):
        for j in range(BOARD_W):
            old_index = currentGrid[i][j]
            new_index = read_grid.data[i * BOARD_W + j]
            if old_index != new_index:
                point = coordinates[i][j]
                if old_index == EMPTY and new_index == PACKAGE:
                    spawn = create_spawn_request(PACKAGE, point)
                    objectPositions[point] = spawn.model_name
                    spawn_client(spawn)
                elif old_index == PACKAGE and new_index == ROBOT:
                    delete_client(objectPositions[point])
                    objectPositions.pop(point)
                    set_model = SetModelState()
                    set_model.model_state.model_name = "sorting_robot"
                    set_model.model_state.pose.position = point
                    set_client(set_model)
                elif old_index == EMPTY and new_index == SHELF:
                    spawn = create_spawn_request(SHELF, point)
                    objectPositions[point] = spawn.model_name
                    spawn_client(spawn)
                elif (old_index == EMPTY or old_index == VISITED) and new_index == ROBOT:
                    if robotSpawned:
                        set_model = SetModelState()
                        set_model.model_state.model_name = "sorting_robot"
                        set_model.model_state.pose.position = point
                        set_client(set_model)
                    else:
                        spawn = create_spawn_request(ROBOT, point)
                        objectPositions[point] = spawn.model_name
                        spawn_client(spawn)
                        robotSpawned = True
                currentGrid[i][j] = new_index
    return UpdateGridResponse(req.grid)

def package_sensor(req):
    """
    Service to emulate a sensor detecting packages within a given grid range.
    """
    sensor_response = SensorResponse()
    packages_detected = []

    # Loop through the grid to find packages within the sensor range
    for i in range(BOARD_H):
        for j in range(BOARD_W):
            if currentGrid[i][j] == PACKAGE:
                packages_detected.append((i, j))

    # Fill the response with detected package coordinates
    sensor_response.detected_objects = []
    for package in packages_detected:
        point = coordinates[package[0]][package[1]]
        sensor_response.detected_objects.append(point)

    return sensor_response


def shelf_sensor(req):
    """
    Service to emulate a sensor detecting shelves within a given grid range.
    """
    sensor_response = SensorResponse()
    shelves_detected = []

    # Loop through the grid to find shelves within the sensor range
    for i in range(BOARD_H):
        for j in range(BOARD_W):
            if currentGrid[i][j] == SHELF:
                shelves_detected.append((i, j))

    # Fill the response with detected shelf coordinates
    sensor_response.detected_objects = []
    for shelf in shelves_detected:
        point = coordinates[shelf[0]][shelf[1]]
        sensor_response.detected_objects.append(point)

    return sensor_response


def execute_pat_task(csp_task):
    """ Executes a task using PAT and returns the result """
    csp_file_path = os.path.join(os.getenv("HOME"), "catkin_ws/src/grid_world/pat", csp_task)
    result = call_pat(csp_file_path)
    return result

def main():
    rospy.init_node('ai_interaction_node')

    # Services and proxies
    bot_location = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    spawn_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    # Initialize coordinates
    for i in range(BOARD_H):
        for j in range(BOARD_W):
            coordinates[i][j].x = i * GRID_WIDTH
            coordinates[i][j].y = j * GRID_WIDTH
            coordinates[i][j].z = 0

    # Advertise services
    rospy.Service('update_grid', UpdateGrid, update_grid)
    rospy.Service('package_sensor', Sensor, package_sensor)
    rospy.Service('shelf_sensor', Sensor, shelf_sensor)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Example of calling a PAT task
        pat_output = call_pat("pat/explore.csp")
        rospy.loginfo("PAT output: %s", pat_output)

        # Main loop logic interacting with PAT and controlling the robot
        # Call the relevant .csp scripts as needed using call_pat()
        if "EXPLORE" in pat_output:
            rospy.loginfo("Exploring the environment")
            call_pat("pat/explore.csp")
        elif "COLLECT_PACKAGES" in pat_output:
            rospy.loginfo("Collecting packages")
            call_pat("pat/collect_packages.csp")
        elif "RETURN_HOME" in pat_output:
            rospy.loginfo("Returning home")
            call_pat("pat/return_home.csp")
        elif "PATH" in pat_output:
            rospy.loginfo("Calculating path")
            call_pat("pat/path.csp")
        
        rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()
