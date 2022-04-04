#!/usr/bin/env python

import rospy, actionlib
from std_msgs.msg import String
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
    entity_name = msg.data
    if entity_name in food_menu:
        location_name = "pizza-table"
    elif entity_name in drink_menu:
        location_name = "drink-table"
    location_coordinate = coordinate_dict[location_name]
    goal = goal_pose(location_coordinate)
    
    print("Destination: ", msg.data, "-Coordinates: ", location_coordinate)
    
    client.send_goal(goal)
    client.wait_for_result()
    

if __name__ == '__main__':
    # init node
    rospy.init_node("pathplanner")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        print("Pathplanner Running")
        # subscribe to ui topic
        sub_location = rospy.Subscriber('location_string', String, loc_cb)
        rate.sleep()