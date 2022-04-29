#!/usr/bin/env python

import rospy, actionlib
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from loc_dictionary import coordinate_dict
from menu_constants import food_menu, drink_menu
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from waiter_bot.msg import Order.msg

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

def order_cb(msg):
    food_item = msg.food.data
    drink_item = msg.drink.data
    name_item = msg.name.data
    order_entry = {
        "name": name_item, 
        "pizza-table": [food_item],
        "drink-table": [drink_item]
    }
    navigate_to_tables(msg, order_entry)

def navigate_to_tables(msg, order_entry):
    print("Start Collecting Items ......")

    # record starting coordinate so that the robot can return to it after collecting items
    odom_msg = rospy.wait_for_message('odom', Odometry)
    message = odom_msg.pose.pose
    ori_pos = message.position
    quat = message.orientation
    original_goal = goal_pose([ori_pos.x, ori_pos.y, ori_pos.z, quat.x, quat.y, quat.z, quat.w])

    if len(order_entry["pizza-table"]) > 0:
        # send location goal to client 
        location_coordinate = coordinate_dict[table]
        goal = goal_pose(location_coordinate)
        client.send_goal(goal)
        print(f"Heading to {table} at coordinates: {location_coordinate}")
        
        while client.get_result() == None:
            #print("navigating while loop ...")
            odom_msg_rough = rospy.wait_for_message('odom', Odometry)
            curr_pos = odom_msg_rough.pose.pose.position
            
            if (abs(curr_pos.x - location_coordinate[0] < 0.5) and abs(curr_pos.y - location_coordinate[1])< 0.5):
                client.cancel_goal()
                print("goal reached")
                break
        
        # tell waiter what items need to be placed on tray
        print(f"I need {order_entry["pizza-table"]}")
        pub_order.publish(msg)
        
        # wait for confirmation that waiter has placed items on tray
        ready_to_go_string = ""
        while ready_to_go_string != "go":
            #print("Waiting for items to be placed on tray...")
            ready_msg = rospy.wait_for_message('order_ready_string', String)
            ready_to_go_string = ready_msg.data 
        print("item placed. Moving on.")

    if len(order_entry["drink-table"]) > 0:
        # send location goal to client 
        location_coordinate = coordinate_dict[table]
        goal = goal_pose(location_coordinate)
        client.send_goal(goal)
        print(f"Heading to {table} at coordinates: {location_coordinate}")
        
        while client.get_result() == None:
            #print("navigating while loop ...")
            odom_msg_rough = rospy.wait_for_message('odom', Odometry)
            curr_pos = odom_msg_rough.pose.pose.position
            
            if (abs(curr_pos.x - location_coordinate[0] < 0.5) and abs(curr_pos.y - location_coordinate[1])< 0.5):
                client.cancel_goal()
                print("goal reached")
                break
        
        # tell waiter what items need to be placed on tray
        print(f"I need {order_entry['drink-table']}")
        pub_order.publish(msg)
        
        # wait for confirmation that waiter has placed items on tray
        ready_to_go_string = ""
        while ready_to_go_string != "go":
            #print("Waiting for items to be placed on tray...")
            ready_msg = rospy.wait_for_message('order_ready_string', String)
            ready_to_go_string = ready_msg.data 
        print("item placed. Moving on.")       

    print("Collected all items. Returning to Patron...")

    client.send_goal(original_goal)
    while client.get_result() == None:
        print("navigating while loop ...")
        odom_msg_rough = rospy.wait_for_message('odom', Odometry)
        curr_pos = odom_msg_rough.pose.pose.position
        print(f"curr_pos: {curr_pos.x}, {curr_pos.y}")
        print(f"goal_pos: {ori_pos.x}, {ori_pos.y}")
        
        if (abs(curr_pos.x - ori_pos.x < 0.5) and abs(curr_pos.y - ori_pos.y)< 0.5):
            client.cancel_goal()
            print("goal reached")
            break

    # inform the patron they have returned with the requested items
    print("Returned to Patron. ")
    print(f"Here are your {order_entry.values()}")
    pub_order.publish(msg)

# init node
rospy.init_node("pathplanner")
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

# subscribe to ui topic
sub_Order = rospy.Subscriber('Order', Order, order_cb)
sub_order = rospy.Subscriber('order_ready_string', String)
pub_order = rospy.Publisher('Delivery', Order, queue_size = 1)
rospy.spin()