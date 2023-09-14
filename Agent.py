import numpy as np
from HPC import *
from RC_agents.basal_ganglia import *
from data_collection.camera_photo import CamPhoto

from geometry_msgs.msg import TwistStamped
from miro_controller.states import *


class Agent(object):
	def __init__(self):
		rospy.init_node('Agent', anonymous=False, log_level=rospy.INFO)
		self.node_name = rospy.get_name()
		rospy.loginfo('--- {}: Initialized robot controller ---'.format(self.node_name))
		self.r = rospy.Rate(10) #10 Hz
		self.pub_navigation = rospy.Publisher('miro/control/cmd_vel', TwistStamped, queue_size=1)
		self.camera = CamPhoto()
		self.HPC = Conv_AE(n_hidden=100)
		self.BG = Basal_Agent()

	def step(self):
		img_left, img_right = self.camera.read_data()
		state = self.HPC.encoder(img_left).detach().to_numpy()
		action = self.BG.run_network(state)
		self.perform_action(action)

	def perform_action(self, action):
		'''
		'Maps action label to action to MiRo'
		'''
		msg = TwistStamped()
		if action == 1:
			msg.twist.linear.x = 5
		elif action == 2:
			msg.twist.linear.x = 0
			msg.twist.angular.x = 3.5
			msg.twist.angular.z = 3.5
		elif action == 3:
			msg.twist.linear.x = 0
			msg.twist.angular.x = 3.5
			msg.twist.angular.z = -3.5
		self.pub_navigation.publish(msg)
