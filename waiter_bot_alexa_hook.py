#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from flask import *
from waiter_bot.msg import Order
from flask_ask import Ask, statement, question, elicit_slot, confirm_slot
from menu_constants import food_menu, drink_menu
import json
import sys

rospy.init_node('alexa_webhook')
order_pub = rospy.Publisher('orders', Order, queue_size=1)
talking_pub = rospy.Publisher('person_talking', String, queue_size=1)


app = Flask(__name__)
#connect flask_ask to the flask endpoint
ask = Ask(app, '/order')

orders = dict()
held_orders = dict()

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
    return question('dont waste my time, just tell me what you want before my battery dies')

# TODO make a few suggestion from the menu.py to show robot alexa deep integration
@ask.intent('ask_menu')
def ask_menu():
    return statement('nothing good')

# report order to the user when complete TODO write call back for order messages sent from navigation
@ask.intent('read_order')
def read_order():
    return statement('you should probably remember what you ordered')

@ask.intent('slow_service')
def slow_service():
    return statement('speed isnt my problem, my problem is between the chair and the keyboard')

# delete an order and send that order to navigation to either stop current move base or remove from itinerary
@ask.intent('cancel_order')
def cancel_order():
    slots = request.get_json()['request']['intent']['slots']
    if 'value' not in slots['name'].keys():
        return elicit_slot('name', 'what was the name')
    name = slots['name']['value']
    if name not in orders.keys():
        return elicit_slot('name', 'i dont have that name, try again, and get it right this time')
    response = str.format('all right {}, all set, dont waste my time again', name)
    order_pub.publish(orders[name])
    del orders[name]
    return statement(response)

# main function that takes order and sends it to navigation for execution
@ask.intent('get_food_type')
def get_food_type():
    if len(orders) > 2:
        return statement('dont I look busy enough? come find me in a minute or two')
    #(request.get_json()['request']['intent'])
    talking_pub.publish('talking')
    slots = request.get_json()['request']['intent']['slots']
    if 'value' not in slots['topping'].keys():
        #print(request.get_json()['request']['intent'])
        return elicit_slot('topping', 'do you want food')  
    if 'value' not in slots['drink'].keys():
        #print(request.get_json()['request']['intent'])
        return elicit_slot('drink', 'do you want a drink?')
    if 'value' not in slots['name'].keys():
        #print(request.get_json()['request']['intent'])
        return elicit_slot('name', 'what is your name')
    if slots['name']['value'] in orders.keys():
        talking_pub.publish('done talking')
        return elicit_slot('name', 'I already have an order with that name, change your name')
    else:
        new_order = Order()
        new_order.food.data = slots['topping']['value']
        new_order.drink.data = slots['drink']['value']
        new_order.name.data = slots['name']['value']
        orders[new_order.name.data] = new_order
        order_pub.publish(new_order)
        return statement('all right hun, now dont go no where, cause Im no good at hide and seek')

if __name__ == '__main__':
    app.run()
    rospy.spin()
    