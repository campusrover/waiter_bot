# waiter_bot
<h1>Team 1 Restaurant Helper Bot </h1>
<h3>Harry and Ben </h3>
<h2> Project  Outline</h2>
On demo day, our project at the very least will be able to take input from the user, either typed in commands (MVP) or voice commands (best-case scenario),<i> (e.g. get me a slice of pepperoni pizza) </i>, then the robot will know where to get the pizza and return to that spot. The robot will use fiducials to localize itself within the map, and use some combination of lidar and the map to plan its path. We found a data set of pizza and beverage orders that we will use to train our natural language understanding model. The data set seems extensive enough to build a model that will generalize to the task. We will likely build the model using entity extraction powered by a CRF. We plan to also use fiducials to represent different pizza types as well and designate an area on the map where the food and beverages will be. The robot will know to go to that general area and use the appropriate fiducial to get the right pizza. If possible, we want to attach an arm to the robot so that it can pick up objects. In addition to this, we may attempt to have multiple robots that can take commands.

<h2> Learning Goals </h2>
We hope to learn more about using maps and fiducials for navigation. As CL students, we hope to learn more about how to build a working natural language processing pipeline for robots. Neither of us have experience with automated speech recognition, so we hope this project will serve as an entry point for that. Lastly, we are wondering if using a map traditionally used for navigation in robotics can also be used in natural language processing to encode world knowledge. 

<h2> Evaluation </h2>
In terms of evaluation, on the robotics side our project involves SLAM and should at the very least be able to navigate between points on the map. On the NLP side, this project has varying levels of possible difficulties. The main goal is to integrate a robotic system with an NLP system with ASR input being more of a user experience issue. 
