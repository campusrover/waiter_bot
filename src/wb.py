#!/usr/bin/env python

import rospy, actionlib, random 
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from loc_dictionary import coordinate_dict
from wander_location_list import wander_locations
from menu_constants import food_menu, drink_menu
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from waiter_bot.msg import Order

# init node
rospy.init_node("wb")
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

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

delivery_waypoints = []
delivery_needed = False 
food_goal = goal_pose(coordinate_dict['pizza-table'])
drink_goal = goal_pose(coordinate_dict['drink-table'])

def order_cb(msg):
    global delivery_waypoints, delivery_needed

    odom_msg = rospy.wait_for_message('odom', Odometry)
    message = odom_msg.pose.pose
    ori_pos = message.position
    quat = message.orientation
    original_goal = goal_pose([ori_pos.x, ori_pos.y, ori_pos.z, quat.x, quat.y, quat.z, quat.w])

    if (len(delivery_waypoints) == 0) or (delivery_waypoints[0] != food_goal):
        delivery_waypoints.append(food_goal)
        delivery_waypoints.append(drink_goal)
    delivery_waypoints.append(goal_pose(original_goal))
    delivery_needed = True

# subscribe to topics
sub_order = rospy.Subscriber('orders', Order, order_cb)
sub_talking = rospy.SubScriber('person_talking', String)
sub_done_talking = rospy.SubScriber('done_talking', String)
sub_picked_up = rospy.Subscriber('order_picked_up', String)

# publish to topic 
pub_order = rospy.Publisher('deliveries', Order, queue_size = 1)

# WANDER state
def wander():
    global delivery_waypoints, delivery_needed

    keep_wandering = True
    while keep_wandering:
        wander_location = random.choice(wander_locations)
        client.send_goal(goal_pose(wander_location))

        while client.get_result() == None:
            odom_msg_rough = rospy.wait_for_message('odom', Odometry)
            curr_pos = odom_msg_rough.pose.pose.position
            
            if (abs(curr_pos.x - wander_location[0] < 0.5) and abs(curr_pos.y - wander_location[1])< 0.5):
                client.cancel_goal()
                print("goal cancel-reached")
                break

            if delivery_needed:
                client.cancel_goal()
                keep_wandering = False
                print("Order placed. Exit WANDER")
                break
    execute()

def execute():
    print("Executing")


# EXECUTE state
# Trigger: 
    # Exit WANDER and enter EXECUTE when Order.msg published to /orders topic
    # Exit EXECUTE and enter WANDER when order_log is empty 
# main method: execute()
# call execute_orderlog 
# cancel current navigation when talking published to /person_talking topic
    # if past_foodtable is False, add new Order.msg to order_log
# resume current navigation when done_talking published to /done_talking topic 

def determine_table(order_log):
    need_food = False
    need_drink = False
    for entry in order_log:
        if entry[0] != None:
            need_food = True
        if entry[1] != None:
            need_drink = True
    return need_food, need_food

def add_order(msg):
    orders.append(msg)
    entry = (msg.food.data, msg.drink.data)
    order_log1.append(entry)

rospy.spin()