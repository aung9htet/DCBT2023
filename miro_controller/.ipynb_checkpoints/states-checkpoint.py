#!/usr/bin/env python3
import os
import cv2
import time
import rospy
import miro_controller.parameters
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion

# ROS messages import
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import Image, Range, JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt32MultiArray

# ROS services import
from std_srvs.srv import Empty

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

    def __init__(self, reward=0):
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
        if parameters.FIXED_MAP == False:
            self.matrix = [[Node(), Node(), Node()],
                        [Node(), Node(), Node()],
                        [Node(), Node(), Node()]]
        else:
            self.matrix_row = []
            # each row has the number of items based on how many columns there is
            for _ in range(parameters.MAP_COLUMN):
                self.matrix_row.append(Node())
            self.matrix = []
            for _ in range(parameters.MAP_ROW):
                self.matrix.append(self.matrix_row)
        self.agent_pos = parameters.STARTING_POSITION
        # self.agent_matrix = lambda x: np.add(x, [1,1]).tolist()

    def add_column(self, direction):
        """
            add_column function will add a column to each of the row in the map
            representation with the Node initialised

            :param direction: `left` for adding column to left and `right` for adding column to right
        """
        for row in self.matrix:
            if direction == "left":
                row = row.insert(0, Node())
            elif direction == "right":
                row = row.append(Node())
            
    def add_row(self, direction):
        """
            add_row function will add a row to each of the column in the map
            representation with the Node initialised

            :param direction: `top` for adding row to top and `bottom` for adding row to bottom
        """
        # create new row with same dimension as the other rows
        new_row = [Node()]
        for item_loop in range(len(self.matrix[0])-1):
            new_row.append(Node())
        if direction == "top":
            self.matrix.insert(0, new_row)
        elif direction =="bottom":
            self.matrix.append(new_row)
    
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
    
    def show_map(self, value = "reward"):
        """
            show map draws a map representation of what the agent has gathered
            The information shown will include the map and agent position
        """
        print("Matrix Representation of Map based on {}", value)
        matrix_str = ""
        for row in self.matrix:
            if value == "reward":
                matrix_str += " | "
                for item in row:
                    matrix_str += str(item.reward) + " | "
                matrix_str += "\n"
        print(matrix_str)
        print("Agent Position: {}",self.agent_pos)

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
                self.rate: The sleeping rate for the rospy code with the refresh rate
                    controlled in parameters.py
                sonar: contains how far an object is from itself
                number_of_steps: The timestep for the model
        """
        rospy.init_node(node_name)
        self.action_dict = {}
        self.position_dict = {'x': None, 'y': None, 'yaw': None}
        self.map_representation = Map_Representation()
        self.bridge = CvBridge()
        self.camera_data = {'left': None, 'right': None}
        self.sonar = None
        self.number_of_steps = 0

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
        self.sonar_sub = rospy.Subscriber(
            topic_base_name + "/sensors/sonar", Range, self.callback_sonar
        )
        self.kinematic_pub = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )
        self.illum_pub = rospy.Publisher(
            topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )
        # rospy.sleep(parameters.SLEEP_TIMER)
        self.rate = rospy.Rate(parameters.REFRESH_RATE)

        # check for connection
        check_connection = False
        while check_connection == False:
            check_connection = True
            for key in self.position_dict:
                if self.position_dict[key] is None:
                    check_connection = False
            for key in self.camera_data:
                if self.camera_data[key] is None:
                    check_connection = False
            if self.sonar is None:
                check_connection = False

        # ROS service processing
        rospy.wait_for_service('/gazebo/reset_world')
        # the following method can be used for resetting the world
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

    def callback_sonar(self, data):
        """
            callback_sonar function is used to update the sonar range data to
            self.sonar

            :param data: rostopic message data received from subscriber
        """
        self.sonar = data.range
        
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
            the following method is used for the agent to work on actionable states
            in each timestep

            :param action_selection: the key for the dictionary to choose action
        """
        self.action_dict[action_selection]()
        self.number_of_steps += 1
        self.map_representation.move(action_selection)
        self.map_representation.show_map()
    
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

    def get_states(self):
        """
            the following function will get states of the world for the user and if 
            required will translate to data needed by the user
        """
        return self.position_dict
    
    def get_timestep(self):
        """
            the following function is supposed to get the number of timesteps taken by
            the agent
        """
        return self.number_of_steps
    
    def reset_environment(self):
        """
            TODO: reset_environment method will reset both the embodied agent and the positional
            matrix representation of itself
        """
        return False
    
    def get_current_agent_node(self):
        """
            The following method gets recorded node data from the agent's known state space
        """
        agent_x = self.map_representation.agent_pos[0]
        agent_y = self.map_representation.agent_pos[1]
        # its rearranged since the first array dimension is the column and the second
        # dimension is the row
        get_node = self.map_representation.matrix[agent_y][agent_x]
        return get_node
    
class Action_State_RL(State_Representation):
    
    def __init__(self, node_name):
        """
            The following class formalises the action and the methods related to it

            Attributes:
                action_dict: The actions need to be added to this dictionary
                neck_side: This parameter makes the neck decide to go either left or right
                neck_timer: This is used to decide when the neck should turn
        """
        super().__init__(node_name)
        self.action_dict['left'] = self.left
        self.action_dict['right'] = self.right
        self.action_dict['straight'] = self.straight
        self.neck_side = None
        self.neck_timer = time.time()

    def check_wall(self, default=True):
        """
            return True when the wall is in front
            return False when the wall is not in front

            :param default: True checks wall nearer and false checks wall further
                            This parameter is used to decide whether the robot gets
                            nearer to the wall to check or just stay at the spot and
                            check for wall
        """
        if default == True:
            if self.sonar < parameters.WALL_RANGE_NEAR:
                return True
            else:
                return False
        else:
            if self.sonar < parameters.WALL_RANGE_FAR:
                return True
            else:
                return False
        
    def right(self, default=True):
        """
            right function moves the miro 90 degrees to the right then move straight
            for 1 meter

            :param default: True will not move the agent from moving straight after
            turning
        """
        vel_cmd = TwistStamped()
        vel_cmd.twist.angular.z = -parameters.TURN_SPEED
        start_yaw = self.position_dict["yaw"]
        rospy.sleep(parameters.SLEEP_TIMER)
        while np.abs(self.position_dict["yaw"] - start_yaw) < parameters.DEGREE_OF_TURN:
            print(self.position_dict["yaw"])
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()
        if not default:
            self.straight()

    def left(self, default = False):
        """
            left function moves the miro 90 degrees to the left then move straight
            for 1 meter

            :param default: True will not move the agent from moving straight after
            turning
        """
        vel_cmd = TwistStamped()
        vel_cmd.twist.angular.z = parameters.TURN_SPEED
        start_yaw = self.position_dict["yaw"]
        rospy.sleep(parameters.SLEEP_TIMER)
        while np.abs(self.position_dict["yaw"] - start_yaw) < parameters.DEGREE_OF_TURN:
            print(self.position_dict["yaw"])
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()
        if not default:
            self.straight()
    
    def straight(self, readjust=False):
        """
            straight function moves the miro straight 0.05m
            for 1 meter
            
            :param readjust: To have the miro move backwards if it finds a wall
                             This function can be used as a backup in case the agent doesnt find a wall in front
        """
        vel_cmd = TwistStamped()
        vel_cmd.twist.linear.x = parameters.STRAIGHT_DISTANCE
        start_x = self.position_dict["x"]
        start_y = self.position_dict["y"]
        rospy.sleep(parameters.SLEEP_TIMER)
        start_time = time.time()
        while (np.abs(self.position_dict["x"] - start_x) < parameters.STRAIGHT_DISTANCE) and (np.abs(self.position_dict["y"] - start_y) < parameters.STRAIGHT_DISTANCE) and self.check_wall == False:
            print("x: " + str(self.position_dict["x"]) + " ,y: " + str(self.position_dict["y"]))
            print((np.abs(self.position_dict["x"] - start_x) < parameters.STRAIGHT_DISTANCE) or (np.abs(self.position_dict["y"] - start_y) < parameters.STRAIGHT_DISTANCE))
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()
            timer = time.time() - start_time # used for calculate how long agent needs to go back
        
        # readjust part of the code
        if readjust == True:
            start_time = time.time()
            # move the agent back
            if self.check_wall() == True:
                print("Err: Wall is in front! Moving back")
                vel_cmd.twist.linear.x = -parameters.STRAIGHT_DISTANCE
                # Move back for the same amount of time moved front
                while time.time() - start_time < timer:
                    self.vel_pub.publish(vel_cmd)
                    self.rate.sleep()

    def get_possible_action(self, reformat=False):
        """
            get_possible_action function gives you a list of action that you may
            be able to choose for moving the agent

            :param reformat: True if you need to check what actions you can do
                             False if you need to get all actions from the list
        """
        possible_actions = {}
        possible_actions['left'] = self.action_dict['left']
        possible_actions['right'] = self.action_dict['right']
        # decide whether to add straight or not based on whether
        # the format wants it to check for wall or not
        add_straight = True
        if reformat is True:
            if self.check_wall(default=False) == True:
                add_straight = False
        if parameters.FIXED_MAP is True:
            if (self.map_representation.agent_pos[2] == 0) and (self.map_representation.agent_pos[1] == 0):
                add_straight = False
            elif (self.map_representation.agent_pos[2] == 1) and (self.map_representation.agent_pos[0] == len(self.map_representation.matrix[0]) - 1):
                add_straight = False
            elif (self.map_representation.agent_pos[2] == 2) and (self.map_representation.agent_pos[1] == len(self.map_representation.matrix) - 1):
                add_straight = False
            elif (self.map_representation.agent_pos[2] == 3) and (self.map_representation.agent_pos[0] == 0):
                add_straight = False
        if add_straight == True:
            possible_actions['straight'] = self.action_dict['straight']
        return possible_actions

    def act_smart(self):
        """
            act_smart is supposed to turn the miro's neck to act as if it is thinking
            when it is replaying
        """
        # turn the neck every decided period
        if (time.time() - self.neck_timer) < parameters.NECK_TIMER:
            # decide which way to turn
            if value == None:
                value = -0.5
            elif value == 0.5:
                value = -0.5
            else:
                value = 0.5
            neck_msg = JointState()
            neck_msg.position = [0.1, 0, value, 0]
            self.kinematic_pub.publish(neck_msg)

    def light_model(self, model_type):
        """
            light_model function is supposed to change the lights based on whether it
            is "MF" for green or "MB" for red.

            :param model_type: Choose whether to behave based on "MF" or "MB"
        """
        color_change = UInt32MultiArray()
        if model_type == "MF":
            # green color
            color = (50, 200, 50)
        elif model_type == "MB":
            # red color
            color = (200, 50, 50)
        else:
            color = (0, 0, 0)
        color = '0xFF%02x%02x%02x'%color
        color = int(color, 16)
        color_change.data = [
            color,
            color,
            color,
            color,
            color,
            color
        ]
        self.illum_pub.publish(color_change)

    def get_reward(self):
        """
            TODO: get_reward method is supposed to give the user with the reward
            values related to this state space based on its vision based
            deduction
        """
        return False
    
# makes the miro move in circles
testing = Action_State_RL("action_pub")
testing.reset_world()
# testing.map_representation.show_map()
# print(testing.map_representation.agent_pos)
# testing.movement('straight')
# testing.movement('straight')
# testing.movement('straight')
# testing.movement('straight')
# testing.movement('straight')
# while not rospy.is_shutdown():
    # testing.get_camera("right")
    # print(np.array(testing.camera_data["left"]).shape)
    
# To Test
# 1: Wall Check
# 2: Light
# 3: Act Smart
# 4: Adjust Model Distances
# 5: Matrix Representation
# 6: Reward