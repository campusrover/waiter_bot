#!/usr/bin/env python

import tkinter as tk
import tkinter.ttk as ttk
from ttkthemes import ThemedStyle
import rospy

from std_msgs.msg import String
from menu_constants import drink_menu, food_menu

def state_cb(msg):
    global current_state
    current_state = msg.data


rospy.init_node("ui")
command_pub = rospy.Publisher("location_string", String, queue_size=1) 
state_sub = rospy.Subscriber("state_string", String, state_cb)
run_order_pub = rospy.Publisher('order_ready_string', String, queue_size=1)
current_state = "INITIALIZING"

item_unavailable = ""
window = tk.Tk()
window.geometry("350x250")
window.title("Robot Waiter")
style = ThemedStyle(window)
style.set_theme("breeze")
order = tk.StringVar()
unavailable_label = ttk.Label(window, text=item_unavailable)
text_field = ttk.Entry(window, textvariable=order)


def publish_order(event=None):
    entities = set(order.get().split())
    entities = list(entities.intersection(drink_menu).union(entities.intersection(food_menu)))
    if entities:
        entities = ' '.join(entities)
        command_pub.publish(entities)
        order_done_button.pack()
        order_done_button.update()
        item_unavailable = ""
    else:
        item_unavailable = "We don't have that"
    text_field.delete(0, len(order.get()))
    unavailable_label.configure(text=item_unavailable)
    unavailable_label.update()

def run_order():
    run_order_pub.publish('go')
    order_done_button.pack_forget()
    order_done_button.update()

order_done_button = ttk.Button(window, text='Order Placed on Tray', command=run_order)
order_done_button.pack_forget()
order_done_button.update()

while not rospy.is_shutdown():
    window.bind('<Return>', publish_order)
    state_label = ttk.Label(window, text=current_state)
    state_label.pack()
    input_label = ttk.Label(window, text="Enter your order")
    input_label.pack()
    text_field.pack()
    submit_button = ttk.Button(window, text="Submit Order", command=publish_order)
    order_done_button.pack_forget()
    submit_button.pack()
    unavailable_label.pack()
    window.mainloop()

