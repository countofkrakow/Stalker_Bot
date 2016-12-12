# Stalker_Bot
Amanda Loh and Chris Nakovski

A bot that uses ROS Laser Scan data to detect legs and people.

### How to Run

### With Bag Files
Note that only Second.bag and Third.bag only have one person to detect in both 

In one terminal:
run:
roscore

In another terminal:
cd to stalker_bot/Stalker_Bot/src when you clone the repo
then do:
python leg_detector.py

In another terminal: 
cd to the src folder of Stalker_Bot

to play a file: put in command:
rosbag play Third.bag

You can put in any bag file for the bag to run: it doesn't even have to be in this git repo: but you have to be in the folder
where it is to play it

### Without Bag Files
Make sure you are accessing the Turtlebot with these commands each with their own terminal:
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_bringup 3dsensor.launch

You can use rostopic echo /scan to make sure you are getting the laser scans

In another terminal:
cd to stalker_bot/Stalker_Bot/src when you clone the repo
then do:
python leg_detector.py

### Some Notes
Make sure you run the python script first before starting the bag file

### Resource Links
https://github.com/jstnhuang/cse481c_tutorials/wiki/How-to-run-the-leg-detector-on-the-Turtlebot

http://wiki.ros.org/leg_detector

http://wiki.ros.org/ROS/Tutorials

https://github.com/wg-perception/people/tree/hydro-devel/leg_detector/src

http://people.idsia.ch/~giusti/perceivingpeople/iros2014.pdf

http://answers.ros.org/question/31366/leg_detector-and-kinect/
