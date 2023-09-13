#!/usr/bin/env python3

import rospy
import time
import random
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range, JointState

class Walker(object):
	def __init__(self):
		rospy.init_node('Walker', anonymous=False, log_level=rospy.INFO)
		self.node_name = rospy.get_name()
		rospy.loginfo('--- {}: Initialized robot controller ---'.format(self.node_name))
	
		self.r = rospy.Rate(10) #10hz
	
		self.pub_navigation = rospy.Publisher('miro/control/cmd_vel', TwistStamped, queue_size=1)
		self.pub_neck_kinem = rospy.Publisher('miro/control/kinematic_joints', JointState, queue_size=1)
	
		rospy.Subscriber('miro/sensors/sonar', Range, self.update_range)
		
		self.range_data = 0

	
	def detect_obstacle(self): #This function moves Miro's head to the left and to the right to make two measures of proximity toward environmental obstacles.
		time.sleep(1)
		obstacle_R = 0
		obstacle_L = 0
		
		value = -0.5
		msg = JointState()
		msg.position = [0.1, 0, value, 0]
		self.pub_neck_kinem.publish(msg)
		
		obstacle_R = self.range_data

		time.sleep(1)

		value = 0.5
		msg = JointState()
		msg.position = [0.1, 0, value, 0]
		self.pub_neck_kinem.publish(msg)
		
		obstacle_L = self.range_data

		print('R obstacle dist = ' + str(obstacle_R))
		print('L obstacle dist = ' + str(obstacle_L))

		time.sleep(1)

		return obstacle_R, obstacle_L
		
		
		
	
	def update_range(self, data): #Callback function for Miro's sonar sensor
		self.range_data = data.range
	
		
		
		
	def walk(self): # Function that implements a random walking mechanism with discrete navigational behavior and avoids obstacles
		msg = TwistStamped()

		actions = [[5, 0],[5, 0],[5, 0],[5, 0],[3.5, 3.5],[3.5, -3.5]]
		choice = random.randint(0,5)
		action_choice = actions[choice]

		if choice <= 3:
			msg.twist.linear.x = action_choice[0]
			msg.twist.linear.z = action_choice[1]
		else:
			msg.twist.angular.x = action_choice[0]
			msg.twist.angular.z = action_choice[1]



		obstacle_threshold = 0.3
		obstacle_R, obstacle_L = self.detect_obstacle()

		if obstacle_L < obstacle_threshold or obstacle_R < obstacle_threshold:
			print('OBSTACLE!!!')
			if obstacle_R > obstacle_L:
				msg.twist.linear.x = 0
				msg.twist.angular.x = actions[4][0]
				msg.twist.angular.z = actions[4][1]
				print('Obstacle to the Right - Turning Left')

			else:
				msg.twist.linear.x = 0
				msg.twist.angular.x = actions[5][0]
				msg.twist.angular.z = actions[5][1]
				print('Obstacle to the Left - Turning Right')

		
		self.pub_navigation.publish(msg)
		
		
		
	def run(self):
		while not rospy.is_shutdown():
			try:
				self.walk()
				time.sleep(1)

			except KeyboardInterrupt:
				break
		
		
		
		
		
walker = Walker()

if __name__ == "__main__":
	walker.run()
