#!/usr/bin/env python

import rospy, actionlib
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from loc_dictionary import coordinate_dict
from menu_constants import food_menu, drink_menu
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def goal_pose(loc_coordinate):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = loc_coordinate[0]
    goal_pose.target_pose.pose.position.y = loc_coordinate[1]
    goal_pose.target_pose.pose.position.z = loc_coordinate[2]
    goal_pose.target_pose.pose.orientation.x = loc_coordinate[3]
    goal_pose.target_pose.pose.orientation.y = loc_coordinate[4]
    goal_pose.target_pose.pose.orientation.z = loc_coordinate[5]
    goal_pose.target_pose.pose.orientation.w = loc_coordinate[6]
    return goal_pose

def loc_cb(msg):
    entities = msg.data
    entities = entities.split()
    location_list = []
    for _ in entities:
        if _ in food_menu:
            location_list.append("pizza-table")
        elif _ in drink_menu:
            location_list.append("drink-table")
    locations_to_goals(location_list)

def locations_to_goals(locationlist):
    counter = 0
    print("Processing a New Batch of Orders ......")
    original_goal = goal_pose([0, 0, 0, 0, 0, 0, 1.0])

    
    odom_msg = rospy.wait_for_message('odom', Odometry)
    message = odom_msg.pose.pose
    pos = message.position
    quat = message.orientation
    original_goal = goal_pose([pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w])

    for location in locationlist:
        counter += 1
        print(f"order {counter} of {len(locationlist)}")
        location_coordinate = coordinate_dict[location]
        goal = goal_pose(location_coordinate)
        print(f"Destination: {location} | Coordinates: {location_coordinate}")
        client.send_goal(goal)
        client.wait_for_result()
    print("done with goal locations")

    client.send_goal(original_goal)
    client.wait_for_result()
    print("done with original locations")

    #Press a button, and will publish the message “go” to topic order_ready_string  

# init node
rospy.init_node("pathplanner")
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

rate = rospy.Rate(1)


# subscribe to ui topic
sub_location = rospy.Subscriber('location_string', String, loc_cb)
sub_order = rospy.Subscriber('order_ready_string', String)
rospy.spin()