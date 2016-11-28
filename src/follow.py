#!/usr/bin/env python
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from people_msgs import PositionMeasurementArray, PositionMeasurement
import rospy

ROT_DELTA = 0.05

class robot_controller:

	def __init__(self):
		rospy.on_shutdown(shutdown)
		rospy.init_node('robot_controller')
		self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
		rospy.Subscriber("people_tracker_measurements", PositionMeasurementArray, self.people_position_callback)
		rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.robot_position_callback)

	def turn_left(self):
		turn_cmd = Twist()
		turn_cmd.linear.x = 0
		turn_cmd.angular.z = radians(-2)
		self.cmd_vel.publish(turn_cmd)

	def turn_right(self):
		turn_cmd = Twist()
		turn_cmd.linear.x = 0
		turn_cmd.angular.z = radians(2)
		self.cmd_vel.publish(turn_cmd)

	def go_forward(self):
		cmd = Twist()
		cmd.linear.x = 0.02
		cmd.angular.z = 0
		self.cmd_vel.publish(cmd)

	def people_position_callback(self, data):

	def robot_position_callback(self, data):

	def shutdown(self):
	    rospy.loginfo("Stop!") 
	    self.cmd_vel.publish(Twist()) 
	    rospy.sleep(1)

if __name__ == '__main__':
	
	controller = robot_controller()	
	while not rospy.is_shutdown():
		pass
