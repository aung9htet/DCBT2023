#!/usr/bin/env python3
import os
import cv2
import rospy
import parameters
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion

# ROS messages import
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped

# class Node:
#     """
#         Nodes that may be required for a 2 dimensional doubly linked list
                       
#                        up_node
#                   __|            |__
#         prev_node __ current_node __ next_node
#                     |            |
#                       down_node

#         Attributes:
#             data: positional data of the world [x, y, yaw], where yaw has 0 for up,
#             1 for right, 2 for down and 3 for left
#             prev, up, down, next: the node data that is linked to the current node
#     """
#     def __init__(self, data):
#         self.data = data
#         self.reward = None
#         self.prev = None
#         self.up = None
#         self.down = None
#         self.next = None
    
#     def set_reward(self, reward):
#         """
#             Set the reward after understanding the environment

#             :param reward: reward likelihood for current state
#         """

#         self.reward = reward
        
# class Map_Representation():
#     """
#         The following code will represent a two dimensional doubly linked list for
#         representation of state space
#     """

#     def __init__(self):
#         self.current_node = Node([0,0,0])
    
#     def move(self):
#         match self.current_node[2]:
#             case 0:
#                 try:
#                     # check if there is a node on the top left side
#                     check_node = self.current_node.prev.up.next
#                     if check_node != None:
#                         check_node.down = self.current_node
#                         self.current_node.up = check_node
#                 except:
#                     print("No prev node at up")
#                 try:
#                     # check if there is a node on the top right side
#                     check_node = self.current_node.next.up.prev
#                     if check_node != None:
#                         check_node.down = self.current_node
#                         self.current_node.up = check_node
#                 except:
#                     print("No next node at up")
                
#                 self.current_node.up = self.Node.
#     def add(self, action):
#         if action == "straight":

class Node:

    def __init__(self, reward=None):
        """
            The following object is created to store current node's data

            Attributes:
                reward: reward probability
        """
        self.reward = reward
        self.wall = False

class Map_Representation():

    def __init__(self):
        """
            The following code will represent a map as a two dimensional list. This data
            will include the agent's positional data as well on the map

            Attributes:
                matrix: The current mapping
                agent_pos: positional data of the agent. The position of the agent at
                    initialisation can be described as follows:
                    [position x on matrix, position y on matrix, yaw]
                    yaw: 0 for up, 1 for right, 2 for down, 3 for left
        """
        self.matrix = [[Node, Node, Node],
                       [Node, Node, Node],
                       [Node, Node, Node]]
        self.agent_pos = [1,1,0]
        # self.agent_matrix = lambda x: np.add(x, [1,1]).tolist()

    def add_column(self, direction):
        """
            add_column function will add a column to each of the row in the map
            representation with the Node initialised

            :param direction: `left` for adding column to left and `right` for adding column to right
        """
        for row in self.matrix:
            if direction == "left":
                row = row.insert(0, Node)
            elif direction == "right":
                row = row.append(Node)
            
    def add_row(self, direction):
        """
            add_row function will add a row to each of the column in the map
            representation with the Node initialised

            :param direction: `top` for adding row to top and `bottom` for adding row to bottom
        """
        # create new row with same dimension as the other rows
        new_row = []
        for i in range(len(self.matrix[0])):
            new_row = new_row.append(Node)
        if direction == "top":
            self.matrix = self.matrix.insert(0, new_row)
        elif direction =="bottom":
            self.matrix = self.matrix.append(new_row)
    
    def change_position(self):
        """
            change_position function will move the agent to another position straight 
            in the map. It will adjust the map to add row and column when needed.

        """
        direction = self.agent_pos[2]
        if direction == 0:
            if self.agent_pos[1] == 0:
                self.add_row(direction="top")
            else:
                self.agent_pos[1] = self.agent_pos[1] - 1
        elif direction == 1:
            if self.agent_pos[0] == (len(self.matrix[0]) - 1):
                self.add_column("right")
                self.agent_pos[0] = self.agent_pos[0] + 1
            else:
                self.agent_pos[0] = self.agent_pos[0] + 1
        elif direction == 2:
            if self.agent_pos[1] == (len(self.matrix) - 1):
                self.add_row(direction="bottom")
                self.agent_pos[1] = self.agent_pos[1] + 1
            else:
                self.agent_pos[1] = self.agent_pos[1] + 1
        elif direction == 3:
            if self.agent_pos[1] == 0:
                self.add_column("left")
            else:
                self.agent_pos[1] = self.agent_pos[1] - 1

    def move(self, action):
        """
            change how the agent moves on the map matrix by using the action

            :param action: the following determines the action of the agent
                            "left" and "right" turns the agent to their respective
                            yaw and move the agent
                            "straight" just move the agent
        """

        if action == 'right':
            self.agent_pos[2] += 1
            if self.agent_pos[2] > 3:
                self.agent_pos[2] == 0
            self.change_position()
        elif action == 'left':
            self.agent_pos[2] -= 1
            if self.agent_pos[2] < 0:
                self.agent_pos[2] = 3
            self.change_position()
        elif action == 'straight':
            self.change_position()
    
    def show_map(self):
        """
            show map draws a map representation of what the agent has gathered
        """
        for row in self.matrix:
            print(row + "\n")

class State_Representation:

    def __init__(self, node_name):
        """
            The following class will be used to formalise the action that can be done
            by the robot

            Sub-class: Action State RL => To make a list of the actions that are do-able
            by the robot
            
            Attributes:
                node_name: The name of the node to be initialised
                action_dict: The key is the action name in str and the value is the
                    method of the action associated to the key
                position_dict: The key is the kind of position data and the value is
                    the related positional data of the robot
                matrix_representation: Map information of the agent's environment.
                    This is updated as an online process.
                camera_data: The camera data stored in dictionary and sorted as 'left'
                    and 'right' camera
                topic_base_name: The name of the robot that is being controlled
                caml_sub: The subcriber for left camera
                camr_sub: The subscriber for right camera
                vel_pub: The publisher for movement
                vel_sub: The subscriber for movement
                self.rate = The sleeping rate for the rospy code with the refresh rate
                    controlled in parameters.py
        """
        rospy.init_node(node_name)
        self.action_dict = {}
        self.position_dict = {'x': 0, 'y': 0, 'yaw': 0}
        self.map_representation = Map_Representation()
        self.bridge = CvBridge()
        self.camera_data = {'left': None, 'right': None}

        # ROS data processing
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        cam_left_topic = "/sensors/caml"
        cam_right_topic = "/sensors/camr"
        self.caml_sub = rospy.Subscriber(
            topic_base_name + cam_left_topic, Image, self.callback_caml
        )
        self.camr_sub = rospy.Subscriber(
            topic_base_name + cam_right_topic, Image, self.callback_camr
        )
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        self.vel_sub = rospy.Subscriber(
            topic_base_name + "/sensors/odom", Odometry, self.callback_vel
        )
        rospy.sleep(parameters.SLEEP_TIMER)
        self.rate = rospy.Rate(parameters.REFRESH_RATE)

    def callback_vel(self, data):
        """
            callback_vel function is first used to translate the subscriber data into 
            euler coordinates which may then be used to update the dictionary of
            coordinates

            :param data: rostopic message data received from subscriber
        """
        orientation = data.pose.pose.orientation
        self.position_dict["x"] = data.pose.pose.position.x
        self.position_dict["y"] = data.pose.pose.position.y
        (_, _, self.position_dict["yaw"]) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

    def callback_caml(self, data):
        """
            callback_camr function is used to get right camera data from the message
            and store them in the camera dictionary

            :param data: rostopic message data received from subscriber
        """
        try:
            # convert ros img data to open cv images
            self.camera_data['left'] = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
           print(e)

    def callback_camr(self, data):
        """
            callback_camr function is used to get right camera data from the message
            and store them in the camera dictionary

            :param data: rostopic message data received from subscriber
        """
        try:
            # convert ros img data to open cv images
            self.camera_data['right'] = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
           print(e)

    def movement(self, action_selection):
        """
            movement function formalizes the actions that are available to use

            :param action_selection: the key for the dictionary to choose action
        """
        self.action_dict[action_selection]()
        self.map_representation.move(action_selection)
    
    def get_camera(self, orientation):
        """
            get_camera function is used for representing camera data into something
            more visual. the function allows you to use either the left or right camera

            :param orientation: allows the user to choose whether to use the 'left' or
                'right' side of a camera
        """
        camera_data = self.camera_data[orientation]
        if not camera_data is None:
            cv2.imshow("Image window", camera_data)
            cv2.waitKey(3)

class Action_State_RL(State_Representation):
    
    def __init__(self, node_name):
        """
            The following class formalises the action and the methods related to it

            Attributes:
                action_dict: The actions need to be added to this dictionary
        """
        super().__init__(node_name)
        self.action_dict['left'] = self.left
        self.action_dict['right'] = self.right
        self.action_dict['straight'] = self.straight

    def right(self):
        """
            right function moves the miro 90 degrees to the right then move straight
            for 1 meter
        """
        vel_cmd = TwistStamped()
        vel_cmd.twist.angular.z = -parameters.TURN_SPEED
        start_yaw = self.position_dict["yaw"]
        rospy.sleep(parameters.SLEEP_TIMER)
        while np.abs(self.position_dict["yaw"] - start_yaw) < parameters.DEGREE_OF_TURN:
            print(self.position_dict["yaw"])
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()
        self.straight()

    def left(self):
        """
            left function moves the miro 90 degrees to the left then move straight
            for 1 meter
        """
        vel_cmd = TwistStamped()
        vel_cmd.twist.angular.z = parameters.TURN_SPEED
        start_yaw = self.position_dict["yaw"]
        rospy.sleep(parameters.SLEEP_TIMER)
        while np.abs(self.position_dict["yaw"] - start_yaw) < parameters.DEGREE_OF_TURN:
            print(self.position_dict["yaw"])
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()
        self.straight()
    
    def straight(self):
        """
            straight function moves the miro straight 0.05m
            for 1 meter
        """
        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = parameters.STRAIGHT_DISTANCE
        start_x = self.position_dict["x"]
        start_y = self.position_dict["y"]
        rospy.sleep(parameters.SLEEP_TIMER)
        while (np.abs(self.position_dict["x"] - start_x) < parameters.STRAIGHT_DISTANCE) or (np.abs(self.position_dict["y"] - start_y) < parameters.STRAIGHT_DISTANCE):
            print("x: " + str(self.position_dict["x"]) + " ,y: " + str(self.position_dict["y"]))
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()


# makes the miro move in circles
testing = Action_State_RL("action_pub")
testing.movement('right')