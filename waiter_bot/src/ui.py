#!/usr/bin/env python

import tkinter as tk
import tkinter.ttk as ttk
from ttkthemes import ThemedStyle
import rospy

from std_msgs.msg import String



rospy.init_node("ui")
command_pub = rospy.Publisher("location_string", String, queue_size=1) 

def publish_selected(menu):
    command_pub.publish(menu)

while not rospy.is_shutdown():
    window = tk.Tk()
    window.geometry("350x250")
    window.title("Robot Waiter")
    style = ThemedStyle(window)
    style.set_theme("breeze")
    menu = tk.StringVar()
    menu.set("...")
    drop = ttk.OptionMenu(window, menu, "Select location", "dining", "bathroom", "pantry", command=publish_selected )
    drop.pack()
    window.mainloop()

