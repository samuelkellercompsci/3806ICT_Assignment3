#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point
from grid_world.srv import UpdateGrid, UpdateGridResponse, Sensor, SensorResponse
from gazebo_msgs.srv import GetModelState, SpawnModel, DeleteModel, SetModelState
import os
import random
import math

# Constants
GRID_WIDTH = 1.0
BOARD_H = 10  # Example height, replace with actual value
BOARD_W = 10  # Example width, replace with actual value
EMPTY, SURVIVOR, HOSTILE, SUB, VISITED = 0, 1, 2, 3, 4

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

# Initialize coordinates
for i in range(BOARD_H):
    for j in range(BOARD_W):
        coordinates[i][j].x = i * GRID_WIDTH
        coordinates[i][j].y = j * GRID_WIDTH
        coordinates[i][j].z = 0

def create_spawn_request(model_type, position):
    spawn = SpawnModel()
    model_dir = os.path.join(os.getenv("HOME"), "catkin_ws/src/grid_world/models/")
    if model_type == SURVIVOR:
        spawn.model_name = "package" + str(numPackages)
        numPackages += 1
        model_path = os.path.join(model_dir, "package/model.sdf")
    elif model_type == HOSTILE:
        spawn.model_name = "shelf" + str(numShelves)
        numShelves += 1
        model_path = os.path.join(model_dir, "shelf/model.sdf")
    elif model_type == SUB:
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
                if old_index == EMPTY and new_index == SURVIVOR:
                    spawn = create_spawn_request(SURVIVOR, point)
                    objectPositions[point] = spawn.model_name
                    spawn_client(spawn)
                elif old_index == SURVIVOR and new_index == SUB:
                    delete_client(objectPositions[point])
                    objectPositions.pop(point)
                    set_model = SetModelState()
                    set_model.model_state.model_name = "sorting_robot"
                    set_model.model_state.pose.position = point
                    set_client(set_model)
                elif old_index == EMPTY and new_index == HOSTILE:
                    spawn = create_spawn_request(HOSTILE, point)
                    objectPositions[point] = spawn.model_name
                    spawn_client(spawn)
                elif (old_index == EMPTY or old_index == VISITED) and new_index == SUB:
                    if robotSpawned:
                        set_model = SetModelState()
                        set_model.model_state.model_name = "sorting_robot"
                        set_model.model_state.pose.position = point
                        set_client(set_model)
                    else:
                        spawn = create_spawn_request(SUB, point)
                        objectPositions[point] = spawn.model_name
                        spawn_client(spawn)
                        robotSpawned = True
                currentGrid[i][j] = new_index
    return UpdateGridResponse(req.grid)

def package_sensor(req):
    # Similar implementation to survivorSensor in C++ code
    pass

def shelf_sensor(req):
    # Similar implementation to hostileSensor in C++ code
    pass

# Advertise services
rospy.Service('update_grid', UpdateGrid, update_grid)
rospy.Service('package_sensor', Sensor, package_sensor)
rospy.Service('shelf_sensor', Sensor, shelf_sensor)

# Keep the node running
rospy.spin()
