# waiter_bot

WaiterBot is a modular solution to automated catering. 

## Team 

Ben Soli (bsoli@brandeis.edu)
Harry Zhu (zhuh@brandeis.edu)

###### How to Use the Code:
1. ```git clone  https://github.com/campusrover/waiter_bot.git``` </br>
2.  ```cm``` </br>
3. Download ngrok and authorize ngrok token </br>
4. ```pip3 install flask-ask``` and ```sudo apt-install ros-noetic-navigation``` </br>
5. Have waiterbot skill on an Alexa device </br>
6. ssh into your robot</br>
7. when you run ```bringup```, make sure your robot is physically located at its initial position on your map. So once you run ```bringup``` and start ```turtlebot3_navigation``` on line 9, don't move you rebot. Otherwise, its odometry will be messed up and ```waiter_bot.py``` will not work as intended </br> 
8. In waiter_bot/src ```bash ngrok_launch.sh``` </br>
9. run command```roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/my_ros_data/yourmap.yaml``` </br>
10. run command```roslaunch waiter_bot waiter_bot.launch``` </br>

WaiterBot aims to use the skills learned in the Autonomous Robot course to imagine the food-service of the future. 

On the front end, WaiterBot uses a node integrated with the Flask Alexa Skills Kit. 
This streamlines the development of Alexa skills backend functionality, allowing programmers to easily create voice applications with multi-turn conversation. 
It is perfect for robotics because this can easily be integrated with a ROS node. This allows Alexa to respond based on the current state of the robot, for example letting a user know if it is currently too busy to take any more orders.
This node can also easily publish messages to other nodes as is it receiving user input, for example, when certain intents are activated, the robot can easily stop its current action
until the the user is done speaking. 

On the backend, the robot uses a look-up table which maps menu items to locations on a physical map. The map was created using SLAM and AMCL. The robot uses move-base from the ros-noetic-navigation package to navigate. 
The idea is designed to be modular in the sense that all the robot needs is a map and a menu with location to know how to navigate around and where to retreive items. 

The backend navigation node receives messages from the Alexa Flask node in the form of Strings and a custom message type called an Order. An Order has three data fields which are Strings referring to a food, a drink, and the users name. 
The name is used to uniquely identify Orders. Once this node receives an Order, it marks the location so it knows where to find the user. With this information, the robot can proceed to gather the users food and drink requests. Once the robot has collected all of the items, it returns to the user.

