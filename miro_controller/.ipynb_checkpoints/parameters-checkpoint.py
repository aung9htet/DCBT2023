import numpy as np

# parameters for the action
STRAIGHT_DISTANCE = 0.25
DEGREE_OF_TURN = (np.pi/2)
TURN_SPEED = (np.pi/12)
STRAIGHT_SPEED = 0.3
WALL_RANGE_NEAR = 0.1
WALL_RANGE_FAR = 0.3
NECK_TIMER = 1

# rospy settings
REFRESH_RATE = 100
SLEEP_TIMER = 0.5

# map representation
FIXED_MAP = False
MAP_ROW = 16
MAP_COLUMN = 16
STARTING_POSITION = [1,1,0]

# model avaliable to use
model_name = ["T_Swift"]