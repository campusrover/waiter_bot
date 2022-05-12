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
delivery_coordinates = []
delivery_needed = False 
food_coordinate = coordinate_dict['pizza-table']
drink_coordinate = coordinate_dict['drink-table']
food_goal = goal_pose(food_coordinate)
drink_goal = goal_pose(drink_coordinate)
order_log = dict()

navigation_tolerance = 0.55

# need a dict of orders, send back when done
# need mapping from order name to places being visted for that order
# cancel order cb should remove the location of that person from the path,
# if all that is left after is the food and or drink table clear the queue, 


person_talking = ""

def order_cb(msg):
    global delivery_waypoints, delivery_coordinates, delivery_needed, order_log
    name = msg.name.data
    food = msg.food.data
    drink = msg.drink.data
    odom_msg = rospy.wait_for_message('odom', Odometry)
    message = odom_msg.pose.pose
    ori_pos = message.position
    quat = message.orientation
    original_coordinate = [ori_pos.x, ori_pos.y, ori_pos.z, quat.x, quat.y, quat.z, quat.w]
    order_path = []
    order_goals = []
    original_goal = goal_pose(original_coordinate)
    if (len(delivery_waypoints) == 0) or (delivery_waypoints[0] != food_goal):
        if food:
            delivery_waypoints.append(food_goal)
            delivery_coordinates.append(food_coordinate)
            order_path.append(food_coordinate)
            order_goals.append(food_goal)
        if drink:
            delivery_waypoints.append(drink_goal)
            delivery_coordinates.append(drink_coordinate)
            order_path.append(drink_coordinate)
            order_goals.append(drink_goal)
    order_path.append(original_coordinate)
    order_goals.append(original_goal)
    order_log[name] = (msg, order_path, order_goals)
    delivery_waypoints.append(original_goal)
    delivery_coordinates.append(original_coordinate)
    delivery_needed = True

def person_talking_cb(msg):
    global person_talking
    person_talking = msg.data

def done_talking_cb(msg):
    global done_talking
    done_talking = msg.data

def cancel_order_cb(msg):
    global delivery_coordinates, delivery_waypoints
    name = msg.name.data
    print('order being canceled')
    print(len(delivery_coordinates))
    path = order_log[name][1]
    per_loc = path[-1]
    goals = order_log[name][2]
    per_goal = goals[-1]
    del order_log[name]
    if per_loc in delivery_coordinates:
        delivery_coordinates.remove(per_loc)
    # clears everything if there are no orders after cancelation
    if not len(order_log):
        delivery_coordinates = []
        delivery_waypoints = []
        delivery_needed = False
        keep_wandering = True
    else:
        if per_goal in delivery_waypoints:
            delivery_waypoints.remove(per_goal)
    print(len(delivery_coordinates))

# subscribe to topics
sub_order = rospy.Subscriber('orders', Order, order_cb)
cancel_order_sub = rospy.Subscriber('canceled_order', Order, cancel_order_cb)
sub_talking = rospy.Subscriber('person_talking', String, person_talking_cb)
sub_done_talking = rospy.Subscriber('done_talking', String, done_talking_cb)
sub_picked_up = rospy.Subscriber('order_picked_up', String)

# publish to topic 
pub_order = rospy.Publisher('deliveries', Order, queue_size = 1)

# WANDER state
def wander():
    print("Wandering")
    global delivery_waypoints, delivery_coordinates, delivery_needed, order_log, person_talking, done_talking

    delivery_needed = False
    person_talking = ""
    done_talking = ""
    delivery_waypoints = []
    delivery_coordinates = []
    order_log = dict()

    keep_wandering = True
    while keep_wandering:
        wander_location = random.choice(wander_locations)
        wander_goal = goal_pose(wander_location)
        client.send_goal(wander_goal)

        while client.get_result() == None:
            odom_msg_rough = rospy.wait_for_message('odom', Odometry)
            curr_pos = odom_msg_rough.pose.pose.position
            
            if (abs(curr_pos.x - wander_location[0] < navigation_tolerance) and abs(curr_pos.y - wander_location[1])< navigation_tolerance):
                client.cancel_goal()
                print("goal cancel-reached")
                break

            if person_talking == 'talking':
                print("talking test")
                client.cancel_goal()
                done_talking = rospy.wait_for_message('done_talking', String)
                print("done talking")
                person_talking = ""
                done_talking = ""
                client.send_goal(wander_goal)

            if delivery_needed:
                client.cancel_goal()
                keep_wandering = False
                print("Order placed. Exit WANDER")
                break
    execute()

def execute():
    print("Executing")
    while len(delivery_waypoints) > 0 or len(order_log) > 0:
        if len(delivery_waypoints) > 0:
            curr_goal = delivery_waypoints.pop(0)
            curr_goal_coordinate = delivery_coordinates.pop(0)
            navigate(curr_goal, curr_goal_coordinate)
    wander()

def navigate(goal, goal_coordinate):
    global delivery_waypoints, delivery_needed, person_talking, done_talking

    print("navigating")
    client.send_goal(goal)
    while client.get_result() == None:
        odom_msg_rough = rospy.wait_for_message('odom', Odometry)
        curr_pos = odom_msg_rough.pose.pose.position
        
        if (abs(curr_pos.x - goal_coordinate[0] < navigation_tolerance) and abs(curr_pos.y - goal_coordinate[1])< navigation_tolerance):
            client.cancel_goal()
            print("goal cancel-reached")
            # if current location is person_location (stored in a dictionary with key name):
            #   send order to be read by alexa
            # del order_log[name]
            if not (goal == food_goal or goal == drink_goal):
                name = list(order_log.keys())[0]
                order = order_log[name][0]
                pub_order.publish(order)
                del order_log[name]
                ready_signal = ""

            ready_msg = rospy.wait_for_message('order_picked_up', String)

            break
        
        if person_talking == 'talking':
            print("talking test")
            client.cancel_goal()
            done_talking = rospy.wait_for_message('done_talking', String)
            print("done talking")
            
            person_talking = ""
            done_talking = ""
            client.send_goal(goal)


# EXECUTE state
# Trigger: 
    # Exit WANDER and enter EXECUTE when Order.msg published to /orders topic
    # Exit EXECUTE and enter WANDER when order_log is empty 
# main method: execute()
# call execute_orderlog 
# cancel current navigation when talking published to /person_talking topic
    # if past_foodtable is False, add new Order.msg to order_log
# resume current navigation when done_talking published to /done_talking topic 


wander()
rospy.spin()