#!/usr/bin/env python

import tkinter as tk
import rospy

from std_msgs.msg import String

rospy.init_node("ui")


while not rospy.is_shutdown():
    window = tk.Tk()
    window.title("GUI")
    label = tk.Label(window, text="Robot Waiter: DOOT DOOT").pack()
    window.mainloop()