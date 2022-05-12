#!/bin/bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src/waiter_bot && ngrok http --subdomain=waiterbot 5000