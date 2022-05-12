#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from flask import *
from waiter_bot.msg import Order
from flask_ask import Ask, statement, question, elicit_slot, confirm_slot
from menu_constants import food_menu, drink_menu
import json
import sys


food_list = ' '.join(food_menu[:-1])
drink_list = " ".join(drink_menu[:-1])
food_list += 'and {}'.format(food_menu[-1])
drink_list += 'and {}'.format(drink_menu[-1])
def deliver_cb(order):
    global held_orders
    held_orders.append(order)

rospy.init_node('alexa_webhook')
order_pub = rospy.Publisher('orders', Order, queue_size=1)
cancel_order_pub = rospy.Publisher('canceled_order', Order, queue_size=1)
talking_pub = rospy.Publisher('person_talking', String, queue_size=1)
order_ready_pub = rospy.Publisher('order_picked_up', String, queue_size=1)
done_talking_pub = rospy.Publisher('done_talking', String, queue_size=1)
delivery_sub = rospy.Subscriber('deliveries', Order, deliver_cb)

app = Flask(__name__)
#connect flask_ask to the flask endpoint
ask = Ask(app, '/order')

orders = dict()
held_orders = []

# prompt the user for order if user simply launches skill
@ask.launch
def start_skill():
    if len(orders) > 2:
        return statement('im a little busy hun, ill get to you eventually')
    return question("What do you want hun?")

# default response for intents app doesn't have coverage for
@ask.intent('AMAZON.FallbackIntent')
def fallback_intent():
    if len(orders) > 2:
        return statement('im trying to work here')
    return question('I didnt get that')

@ask.intent('AMAZON.CancelIntent')
def cancel_intent():
    return statement('OK')

@ask.intent('thank_you')
def thank_you():
    if len(held_orders):
        order = held_orders.pop(0)
        name = order.name.data
        food = order.food.data
        drink = order.drink.data
        del orders[name]
        order_ready_pub.publish('go')
        return statement("I live to serve")
    return statement('for what?')
@ask.intent('ask_menu')
def ask_menu():
    return question('to eat we have {} and to drink we have {}  do you want something?'.format(food_list, drink_list))

# rebuild this dialog
@ask.intent('order_ready')
def order_ready():
    order_ready_pub.publish('go')
    return statement('ok, ill take this over')

@ask.intent('take_trash')
def take_trash():
    return statement('the trash robot is over there')

@ask.intent('deliver_order')
def deliver_order():
    if len(held_orders):
        order = held_orders[0]
        name = order.name.data
        food = order.food.data
        drink = order.drink.data
        return question('hi {}, I have {} and {}'.format(name, food, drink))
    return statement('I dont have any orders right now')


# report order to the user 
@ask.intent('read_order')
def read_order():
    if len(orders):
        response = ''
        for name in orders:
            order_string = 'I need a {} and {} for {} '.format(orders[name].food.data, orders[name].drink.data, name)
            response += order_string
        return statement(response)
    else:
        return statement('no orders')

@ask.intent('slow_service')
def slow_service():
    return statement('speed isnt my problem, my problem is between the chair and the keyboard')

# delete an order
@ask.intent('cancel_order')
def cancel_order():
    if not len(orders):
        return statement('there are no orders')
    slots = request.get_json()['request']['intent']['slots']
    talking_pub.publish('talking')
    if 'value' not in slots['name'].keys():
        return elicit_slot('name', 'what was the name')
    name = slots['name']['value']
    if name not in orders.keys():
        return elicit_slot('name', 'i dont have that name, try again, and get it right this time')
    response = str.format('all right {}, all set, dont waste my time again', name)
    cancel_order_pub.publish(orders[name])
    done_talking_pub.publish('done talking')
    del orders[name]
    return statement(response)

# cancel all orders, only used for debug purposes
@ask.intent('cancel_all_orders')
def cancel_all_orders():
    orders.clear()
    return statement('sounds good less work for me')

# main function that takes order and sends it to navigation for execution
@ask.intent('take_order')
def get_food_type():
    if len(orders) > 2:
        return statement('dont I look busy enough? come find me in a minute or two')
    talking_pub.publish('talking')
    slots = request.get_json()['request']['intent']['slots']
    new_order = Order()
    food = None
    drink = None
    if slots['food']['confirmationStatus'] == "NONE" and 'value' not in slots['food']:
        return confirm_slot('food', 'do you want food?')
    if 'value' not in slots['food'].keys() and slots['food']['confirmationStatus'] == "CONFIRMED":
        return elicit_slot('food', 'ok so tell me what you want')
    if slots['food']['confirmationStatus'] != "DENIED" and slots['food']['value'] not in food_menu:
        return confirm_slot('food', 'we dont have {} do you want something else to eat'.format(slots['food']['value']))
    if slots['food']['confirmationStatus'] == 'DENIED':
        food = ''
        new_order.food.data = food
    if slots['drink']['confirmationStatus'] == "NONE" and 'value' not in slots['drink']:
        return confirm_slot('drink', 'do you want a drink?')
    if 'value' not in slots['drink'].keys() and slots['drink']['confirmationStatus'] == "CONFIRMED":
        return elicit_slot('drink', 'ok so tell me what you want') 
    if slots['drink']['confirmationStatus'] != "DENIED" and slots['drink']['value'] not in drink_menu:
        return confirm_slot('drink', 'we dont have {} do you want something else to drink?'.format(slots['drink']['value']))
    if slots['drink']['confirmationStatus'] == "DENIED":
        drink = ''
        new_order.drink.data = drink
    if food == '' and drink == '':
        done_talking_pub.publish('done talking')
        return statement('ok then, bye')    
    if 'value' not in slots['name'].keys():
        return elicit_slot('name', 'what is your name')
    if slots['name']['value'] in orders.keys():
        return elicit_slot('name', 'I already have an order with that name, change your name')
    else:
        done_talking_pub.publish('done talking')
        if food != '':
            new_order.food.data = slots['food']['value'] if 'value' in slots['food']  else ''
        if drink != '':
            new_order.drink.data = slots['drink']['value'] if 'value' in slots['drink'] else ''
        new_order.name.data = slots['name']['value']
        orders[new_order.name.data] = new_order
        order_pub.publish(new_order)
        return statement('all right, now dont go no where, be right back {}'.format(new_order.name.data))

if __name__ == '__main__':
    app.run()
    rospy.spin()
    
