import matplotlib
import numpy as np
from typing import Tuple
import random
import pandas as pd
import pickle
import os
import matplotlib.pyplot as plt


class Env:
    """
    The environment class will contain all information regarding the environment, as well as it will be responsible for
    the generation of observations for the agent by reacting to the actions of the former.

    The environment is created independently of the agent, thus it can be easily replaced by a different class, or
    further subclasses may be introduced with ease, as long as the communication between it and the agent remains
    undisturbed.
    """

    def __init__(self, **kwargs):
        """
        General constructor of the environment, simply defining the most basic elements of it
        """
        # The encoding of the entire action space (all possible actions, m dimensions)
        self._act = {}
        # The encoding of the maze -- 0 represents a wall, a number represents an available state
        self._maze = np.array([])
        # What is the value of each state defined above
        self._reward = np.array([])
        # What is the likelihood of getting a reward in each state
        self._reward_prob = np.array([])
        # Where te agent is right now
        self._agent_pos = np.array([])
        # Are there any forbidden actions (following the coding of _act) -- for every state specify a vector of
        # length m. 0 means no restriction, 1 means forbidden action in said state
        self._restrict = np.array([])
        # Probability of slipping (stochastic transitioning)
        self._slip_prob = 0
        # About storing the data
        self._save_env = False
        self._events = None
        return

    # Hidden functions for several upkeep purposes

    def __restrict_walls__(self) -> None:
        """
        Restricts bumping into a wall, let that be explicit 0 or just out of bounds
        """
        for x in range(self._maze.shape[0]):
            for y in range(self._maze.shape[1]):
                if self._maze[x, y] >= 0:
                    for u in self._act:
                        [x_prime, y_prime] = self.__next_state__(x, y, u)
                        if self.__check_out_of_bounds__(x_prime, y_prime) or self._maze[x_prime, y_prime] == -1:
                            self._restrict[x, y, u] = 1

    def __check_out_of_bounds__(self, x: int, y: int) -> bool:
        """
        See if an (x, y) coordinate pair is out of bounds on the map, or is if a forbidden filed (i.e. wall)
        :param x: x coordinate
        :param y: y coordinate
        :return: we are out of bounds (True) or not (False)
        """
        if x < 0 or x >= self._maze.shape[0] or y < 0 \
                or y >= self._maze.shape[1] or self._maze[x, y] == -1:
            return True
        return False

    def __next_state__(self, x: int, y: int, u: int) -> np.ndarray:
        """
        Tells us the label and the coordinates of the next state if we take action u in state s (stays in s if the
        action is impossible)
        :param x: the x coordinate we're in
        :param y: the y coordinate we're in
        :return: the coordinates of the arrival state
        """
        [x_prime, y_prime] = np.array([x, y]) + self._act[u]
        return np.array([x_prime, y_prime]).astype(int)

    def __slip__(self, x: int, y: int, x_prime: int, y_prime: int) -> np.array:
        """
        In case of a slippery maze, this function will implement how slip should happen. The basic idea is that we take
        a non-forbidden step from s_prime that doesn't lead us back to s. Watch out, this can happen recursively!
        :param x, y: starting state
        :param x_prime, y_prime: arrival state before slipping
        :return: arrival state after slipping in coordinates
        """
        u_poss = self.possible_moves(self._maze[x_prime, y_prime])
        u_poss_filt = np.copy(u_poss)  # This is the actual u_poss without the action(s) that'd take us back

        # Getting rid of a move that would possibly take us back
        for u in u_poss:
            if np.all(np.array([x, y]) == self.__next_state__(x_prime, y_prime, u)):
                np.delete(u_poss_filt, u_poss_filt == u)

        # Taking the random step
        u = np.random.choice(u_poss_filt)
        [x_fin, y_fin] = self.__next_state__(x_prime, y_prime, u)

        # If we were to go out of bounds, or bump into a wall, stay in place instead:
        if self.__check_out_of_bounds__(x_fin, y_fin):
            x_fin, y_fin = x_prime, y_prime

        return np.array([x_fin, y_fin]).astype(int)

    def __save_step__(self, **kwargs) -> None:
        """
        Saves the current state of the maze by adding a row to the _events memory.
        :param kwargs agent_type: which agent took this step ['MF' or 'MB']
        :return:
        """
        agent_type = kwargs.get('agent_type', None)
        if not self._save_env:
            return

        # 1) Which step are we at
        step = 0
        if self._events.shape[0] > 0:
            step = self._events['iter'].iloc[-1] + 1

        # 2) Format the event to store (we might have more reward columns than needed)
        # iter, agent_type, agent_pos_x, agent_pos_y, rew_pos_x0, rew_pos_y0, rew_val0, rew_proba0, ...
        event = np.ones(self._events.shape[1]) * -1
        event[0] = step
        event[1] = float(agent_type == 'MB')
        agent = np.argwhere(self._agent_pos == 1)
        event[2] = agent[0, 0]
        event[3] = agent[0, 1]
        rewards = np.argwhere(self._reward > 0)
        for rew_idx in range(rewards.shape[0]):
            event[4 + rew_idx * 4] = rewards[rew_idx, 0]
            event[4 + rew_idx * 4 + 1] = rewards[rew_idx, 1]
            event[4 + rew_idx * 4 + 2] = self._reward[rewards[rew_idx, 0], rewards[rew_idx, 1]]
            event[4 + rew_idx * 4 + 3] = self._reward_prob[rewards[rew_idx, 0], rewards[rew_idx, 1]]

        # 3) Add it to the table
        self._events = pd.concat([self._events, pd.DataFrame([event], columns=self._events.columns)])
        return

    def __overwrite_step__(self, x: int, y: int) -> None:
        """
        Overwrites the last stored memory in case the agent was moved without a step having taken place.
        :param x: New x coordinate of the agent
        :param y: New y coordinate of the agent
        :return:
        """
        if self._save_env:
            self._events['agent_pos_x'].iloc[-1] = x
            self._events['agent_pos_y'].iloc[-1] = y
        return

    # Getters that will communicate towards the agent

    def state_num(self) -> int:
        """
        Returns the number of total states possible in the environment, where each state means a separate location in
        the maze

        :return: number of possible states
        """
        return self._maze.max().max() + 1

    def act_num(self) -> int:
        """
        Returns the maximum number of possible actions within the maze for any state.

        :return: max number of possible actions
        """
        return len(self._act)

    def maxrew(self) -> float:
        """
        Returns the maximal obtainable reward from the maze

        :return: max of reward
        """
        return self._reward.max()

    # Communication towards the agent

    def curr_state(self) -> int:
        """
        Returns the current coordinates (as per understood by the agent) of the agent

        :return: current state of the agent
        """
        return self._maze[self._agent_pos.astype(bool)][0]

    def possible_moves(self, s: np.ndarray) -> np.ndarray:
        """
        Given a state of query, it computes all the possible available actions, taking into consideration whether
        movement is restricted or not, and whether the agent can try and bump into walls or not

        :param s: current state label, as understood by the agent
        :return: a numpy array of possible actions to choose from (labels follow those of self._act)
        """
        s = s[0]
        [x, y] = np.argwhere(self._maze == s)[0]
        moves = np.array(range(len(self._act)))
        moves = moves[~self._restrict[x, y, :].astype(bool)]
        return moves.astype(str)

    # And receiving communication from the agent

    def step(self, s: np.ndarray, u: str, **kwargs) -> Tuple[np.ndarray, float, bool]:
        """
        Performs a step from state s (as per designated by the agent), taking action u (as per chosen in advance), and
        returns the observed outcome.
        If the action would drive the agent out of bounds or into the wall, the agent stays in place
        If the environment is slippery, the agent might slip
        Every time we take a step, the

        :param s: state label, as per understood by the agent
        :param u: action label, following the logic of self._act
        :param kwargs MF_winner: did the MF algorithm make this step [True] or the MB [False]
        :return: new state label as per understood by the agent (int), and corresponding reward (float)
        """
        # Let's remove the agent from the starting state
        s = s[0]
        u = int(u)

        [x, y] = np.argwhere(self._maze == s)[0]
        self._agent_pos[x, y] = 0

        # Then see where we land
        [x_prime, y_prime] = self.__next_state__(x, y, u)

        # If we were to go out of bounds, or bump into a wall, stay in place instead:
        if self.__check_out_of_bounds__(x_prime, y_prime):
            x_prime, y_prime = x, y

        # Then we might slip
        if np.random.uniform(0, 1) < self._slip_prob:
            [x_prime, y_prime] = self.__slip__(x, y, x_prime, y_prime)

        # Arriving at our final destination in the environment
        s_prime = self._maze[x_prime, y_prime]
        self._agent_pos[x_prime, y_prime] = 1

        # Generating reward
        rew = 0
        if random.uniform(0, 1) < self._reward_prob[x_prime, y_prime]:
            rew = self._reward[x_prime, y_prime]

        # Seeing if we finished
        done = False
        if rew > 0:
            done = True

        # Saving
        MF_winner = kwargs.get('MF_winner', None)
        agent_type = None
        if MF_winner is not None:
            if MF_winner:
                agent_type = 'MF'
            else:
                agent_type = 'MB'
        self.__save_step__(agent_type=agent_type)
        return np.array([s_prime]), rew, done

    def place_reward(self, reward_state: int, reward_val: float, reward_prob: float) -> None:
        """
        Places the reward.
        :param reward_prob: Probability of said reward
        :param reward_val: Value of this reward
        :param reward_state: Where this reward should be placed (state-space representation)
        :return: -
        """
        [x, y] = np.argwhere(self._maze == reward_state)[0]
        # Where is the reward and how big is it?
        self._reward[x, y] = reward_val
        # What is the likelihood of getting a reward
        self._reward_prob[x, y] = reward_prob

        # Call the toggle_save, because in case we are saving, adding a new reward means we need to extend the storage
        self.toggle_save(save_on=self._save_env)
        return

    def reset_reward(self) -> None:
        """
        Resets the reward to zero.

        :return: -
        """
        self._reward = np.zeros(self._maze.shape)
        self._reward_prob = np.zeros(self._maze.shape)
        return

    def place_agent(self, init_state: int) -> np.ndarray:
        """
        A function to place the agent onto state init_state. If saving is on this function will overwrite the location
        of the agent in the last row of the memory.
        :param init_state: the state (understood by the agent) where we should be placed
        :return: the initial state
        """
        [x, y] = np.argwhere(self._maze == init_state)[0]

        # Remove the agent from its old position
        self._agent_pos = np.zeros(self._maze.shape)
        # And then place it onto the new
        self._agent_pos[x, y] = 1

        # Take care of saving by overwriting the last element
        self.__overwrite_step__(x, y)
        return np.array([self._maze[x, y]])

    # About saving
    def toggle_save(self, **kwargs) -> None:
        """
        Toggles save. If the environment was saving its status so far, it sops doing so. Otherwise, it begins to do so,
        by already storing a snapshot of the current state as well.
        If a new reward has been added recently, we'll increase the size of the memory to accomodate it.
        :param kwargs:
            save_on: If instead of toggling, we want to make sure to turn it on [True] or off [False], we can
        :return:
        """
        save_on = kwargs.get('save_on', not self._save_env)
        if save_on:
            try:
                if self._events.shape[1] <= (self._reward > 0).sum().sum() * 4 + 4:  # We have recently added new reward
                    # The idx of the last reward we have as a col
                    last_column = int((self._events.shape[1] - 4) / 4 - 1)
                    # The idx+1 of the very last reward that we added
                    extend_to = int((self._reward > 0).sum().sum())
                    col_names = [f'{variable_name}{variable_num}' for variable_num in
                                 range(last_column + 1, extend_to)
                                 for variable_name in ['rew_pos_x', 'rew_pos_y', 'rew_val', 'rew_proba']]
                    for col_name in col_names:
                        self._events[col_name] = np.ones(self._events.shape[0]) * -1  # We fill up with -1s
            except AttributeError:  # There is no such thing as _events
                col_names = [f'{variable_name}{variable_num}' for variable_num in range((self._reward > 0).sum().sum())
                             for variable_name in ['rew_pos_x', 'rew_pos_y', 'rew_val', 'rew_proba']]
                self._events = pd.DataFrame(columns=['iter', 'agent_type', 'agent_pos_x', 'agent_pos_y', *col_names])
            if not self._save_env:
                self._save_env = True
                self.__save_step__()
        else:
            self._save_env = False

    def dump_env(self, **kwargs) -> None:
        """
        Saves everything that we have stored into 2 different files: one for the environment, and one for the events.
        :param kwargs:
            path: [str] the path to save the document. If no path is defined then the current working folder will be
                used
            label: [str] an additional label to add at the end of the output file name.
        :return:
        """
        path = kwargs.get('path', None)
        if path is not None:
            path = f'{path}/'
            if not os.path.isdir(path):
                os.mkdir(path)
        else:
            path = './'
        label = kwargs.get('label', None)
        if label is not None:
            label = f'_{path}'
        else:
            label = ''

        # 1) Save the whole environment
        file = open(f'{path}environment{label}.txt', 'wb')
        pickle.dump(self.__dict__, file, 2)
        file.close()

        # 2) Save the events
        try:
            self._events.to_csv(f'{path}environment{label}.csv', sep=',', index=False, encoding='utf-8')
        except AttributeError:
            print('Note: This environment does not store the transpired events, no .csv generated.')

    def load_env(self, file_name: str, **kwargs):
        """
        Loads a previously saved environment
        :param file_name: the name of the environment file [txt]
        :param kwargs:
            path: path to the file. If nothing is specified we'll be looking in the working folder
        :return:
        """
        path = kwargs.get('path', None)
        if path is not None:
            path = f'{path}/'
            if not os.path.isdir(path):
                raise FileNotFoundError(f'No directory named {path}')
        else:
            path = './'

        if os.path.isfile(f'{path}{file_name}'):
            file = open(f'{path}{file_name}', 'rb')
            tmp_dict = pickle.load(file)
            file.close()
            self.__dict__.update(tmp_dict)
        else:
            raise FileNotFoundError(f'No file named {file_name}')


class dTmaze(Env):
    """
    A child class to env, where we can specify different mazes, without losing the functions already used in the parent
    class. The maze and all of its properties will have to be initialized as matrices contained in np arrays

    dT maze stands for double T maze
    """

    def __init__(self, **kwargs):
        """
        Constructor of the double T maze class

        :param kwargs:  forbidden_walls -- is the agent allowed to choose to bump into the wall
                        restricted_dT   -- creates a restricted double-T maze where we can only walk in one direction
                        slip_prob       -- the probability of slipping after a step
        """
        # Handling the potential kwargs
        forbidden_walls = kwargs.get('forbidden_walls', False)
        restricted_dT = kwargs.get('restricted_dT', False)
        slip_prob = kwargs.get('slip_prob', 0)

        # Setting everything up so that we have a double-T maze
        Env.__init__(self)
        # The encoding of the possible actions: {0: up, 1: right, 2: down, 3: left}
        self._act = {0: np.array([-1, 0]), 1: np.array([0, 1]), 2: np.array([1, 0]), 3: np.array([0, -1])}
        # The maze itself
        self._maze = np.array([[0, 1, 2, 3, 4, 5, 6, 7, 8],
                               [9, -1, -1, -1, 10, -1, -1, -1, 11],
                               [12, -1, 13, 14, 15, -1, -1, -1, 16],
                               [17, -1, -1, 18, -1, -1, -1, -1, 19],
                               [20, -1, -1, 21, -1, -1, -1, -1, 22],
                               [23, 24, 25, 26, 27, 28, 29, 30, 31]])
        # Transitions
        self._slip_prob = slip_prob
        # Where is the reward
        self._reward = np.zeros(self._maze.shape)
        # What is the likelihood of getting a reward
        self._reward_prob = np.zeros(self._maze.shape)
        # Where do we usually start from
        self._agent_pos = np.zeros(self._maze.shape)
        # Are there any forbidden actions (following the coding of _act)
        if restricted_dT:
            # In this case I want to simply test what happens if I restrict going backwards
            self._restrict = np.array(
                [[[0, 1, 0, 0], [0, 1, 0, 0], [0, 1, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1],
                  [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
                 [[1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0],
                  [0, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]],
                 [[1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0],
                  [0, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]],
                 [[1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0], [0, 0, 0, 0],
                  [0, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]],
                 [[1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0], [0, 0, 0, 0],
                  [0, 0, 0, 0], [0, 0, 0, 0], [1, 0, 0, 0]],
                 [[1, 0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 1], [0, 1, 0, 1], [0, 1, 0, 0], [0, 1, 0, 0],
                  [0, 1, 0, 0], [0, 1, 0, 0], [1, 0, 0, 0]]])
        else:
            self._restrict = np.zeros((self._maze.shape[0], self._maze.shape[1], len(self._act)))
        # Are the walls restricted
        if forbidden_walls:
            self.__restrict_walls__()

        return


class PlotterEnv(Env):
    """
    A child class of Env that can load in the event history of an agent and plot it
    """

    def __init__(self, file_name: str, **kwargs):
        """
        This class will only ever be used to plot previous results, thus we can only call it by loading a file
        :param file_name: the name of the environment file [csv]
        :param kwargs:
            path: path to the file. If nothing is specified we'll be looking in the working folder
        :return:
        """
        Env.__init__(self)
        self._agent_events = None

        path = kwargs.get('path', None)
        self.load_env(file_name, path=path)
        return

    # Internal methods of the plotter
    def __event_to_img__(self, values: pd.core.frame.DataFrame) -> np.ndarray:
        """
        Takes a row from a pandas dataframe, each column of ot containing a value corresponding a state. This row is
        then converted into a numpy array where these values are projected onto the actual maze.
        :param values:
        :return:
        """
        values = values.to_numpy()
        image = np.zeros(self._maze.shape)
        image[self._maze >= 0] = values
        return image

    def __status_to_image__(self, it: int) -> np.ndarray:
        """
        It will produce an array reflecting the status of the maze in iteration it. The array will follow the following
        conventions: wall = 0, path = 1, reward = 2 (irrelevant of value), agent = 3. If the agent is in a rewarded
        state, the state will have a value of 3 (agent)
        :param it: the iteration we are in
        :return:
        """
        # wall = 0, path = 1, reward = 2, MFagent = 3, MBagent = 4
        image = np.zeros(self._maze.shape)
        image[self._maze >= 0] = 1
        reward_num = int((self._events.shape[1] - 3) / 4)
        for rew_idx in range(reward_num):
            image[int(self._events[f'rew_pos_x{rew_idx}'].iloc[it]),
            int(self._events[f'rew_pos_y{rew_idx}'].iloc[it])] = 2
        if self._events['agent_type'].iloc[it] == 0:
            image[int(self._events['agent_pos_x'].iloc[it]), int(self._events['agent_pos_y'].iloc[it])] = 3
        else:
            image[int(self._events['agent_pos_x'].iloc[it]), int(self._events['agent_pos_y'].iloc[it])] = 4
        return image

    def __replay_to_image__(self, curr_image: np.ndarray, row_idx: int) -> np.ndarray:
        """
        Takes the last array representing the replayed states (if no replay had taken lace earlier, we simply use an
        array of zeros) and based on the current row_idx (not iter, not step), we add the last replay to this maze
        :param curr_image: The array depiocting the last replay step
        :param row_idx: the row idx in the agent event memory table the replay of which we want to depict
        :return:
        """
        max_val = curr_image.max().max()
        s = self._agent_events['s'].iloc[row_idx]
        [x, y] = np.argwhere(self._maze == s)[0]
        curr_image[x, y] = max_val + 1
        s_prime = self._agent_events['s_prime'].iloc[row_idx]
        [x, y] = np.argwhere(self._maze == s_prime)[0]
        curr_image[x, y] = max_val + 1
        return curr_image

    # Function to load in the data in question
    def load_events(self, file_name: str, **kwargs):
        """
        Loads the steps of an agent
        :param file_name: the name of the agent file [csv]
        :param kwargs:
            path: path to the file. If nothing is specified we'll be looking in the working folder
        :return:
        """
        path = kwargs.get('path', None)
        if path is not None:
            path = f'{path}/'
            if not os.path.isdir(path):
                raise FileNotFoundError(f'No directory named {path}')
        else:
            path = './'

        if os.path.isfile(f'{path}{file_name}'):
            self._agent_events = pd.read_csv(f'{path}{file_name}')
        else:
            raise FileNotFoundError(f'No file named {file_name}')

    def plot_events(self):
        """
        Plots the events of the experiment in an animated fashion. It uses 2 distinct plots: one for the maze, the
        replay and the Q values, the other one for the H_rew and the H_trans values.
        :return:
        """
        # 0) Preparing the dataframes -- we need the max Q value for each state, and (as of now) the mean H value
        Q_vals = pd.DataFrame()
        H_rew_vals, H_trans_vals = pd.DataFrame(), pd.DataFrame()
        for s_idx in range(self.state_num()):
            if f'Q_{s_idx}_0' in self._agent_events.columns:
                cols = [f'Q_{s_idx}_{u_idx}' for u_idx in range(self.act_num())]
                Q_vals[f'Q_{s_idx}'] = self._agent_events[cols].max(axis=1)
            else:
                Q_vals[f'Q_{s_idx}'] = pd.DataFrame(np.zeros(self._agent_events.shape[0]),
                                                    columns=[f'Q_{s_idx}'])

            if f'H_rew_{s_idx}_0' in self._agent_events.columns:
                cols = [f'H_rew_{s_idx}_{u_idx}' for u_idx in range(self.act_num())]
                H_rew_vals[f'H_rew_{s_idx}'] = self._agent_events[cols].mean(axis=1)
            else:
                H_rew_vals[f'H_rew_{s_idx}'] = pd.DataFrame(np.zeros(self._agent_events.shape[0]),
                                                            columns=[f'H_rew_{s_idx}'])

            if f'H_trans_{s_idx}_0' in self._agent_events.columns:
                cols = [f'H_trans_{s_idx}_{u_idx}' for u_idx in range(self.act_num())]
                H_trans_vals[f'H_trans_{s_idx}'] = self._agent_events[cols].mean(axis=1)
            else:
                H_trans_vals[f'H_trans_{s_idx}'] = pd.DataFrame(np.zeros(self._agent_events.shape[0]),
                                                                columns=[f'H_trans_{s_idx}'])
        Q_max = Q_vals.to_numpy().max().max()
        H_rew_max = H_rew_vals.to_numpy().max().max()
        H_trans_max = H_trans_vals.to_numpy().max().max()

        # 1) Preparing the Q plots and the H plots
        plt.ion()
        fig_Q, ax_Q = plt.subplots(nrows=1, ncols=3, figsize=(15, 4))
        ax_Q[0].set_title("Map")
        ax_Q[1].set_title("Replay")
        ax_Q[2].set_title("Q_values")
        curr_maze = self.__status_to_image__(0)
        curr_replay = np.zeros(self._maze.shape)
        curr_Q = self.__event_to_img__(Q_vals.iloc[0])
        axim_Q = np.array([ax_Q[0].imshow(curr_maze, vmin=0, vmax=4),
                           ax_Q[1].imshow(curr_replay),
                           ax_Q[2].imshow(curr_Q, vmin=0, vmax=Q_max)])
        axim_Q[1].autoscale()  # This will have to be done in every step if we want the old replay steps to fade away

        fig_H, ax_H = plt.subplots(nrows=1, ncols=2, figsize=(10, 4))
        ax_H[0].set_title("Reward entropy")
        ax_H[1].set_title("Transition entropy")
        curr_H_rew = self.__event_to_img__(H_rew_vals.iloc[0])
        curr_H_trans = self.__event_to_img__(H_trans_vals.iloc[0])
        axim_H = np.array([ax_H[0].imshow(curr_H_rew, vmin=0, vmax=H_rew_max),
                           ax_H[1].imshow(curr_H_trans, vmin=0, vmax=H_trans_max)])

        txt = np.empty((*self._maze.shape, 3), dtype=matplotlib.text.Text)  # txt will appear for the Q and the H values
        for idx_x in range(txt.shape[0]):
            for idx_y in range(txt.shape[1]):
                txt[idx_x, idx_y, 0] = ax_Q[2].text(idx_y, idx_x, f"{curr_Q[idx_x, idx_y]: .2f}",
                                                    ha="center", va="center", color="w")
                txt[idx_x, idx_y, 1] = ax_H[0].text(idx_y, idx_x, f"{curr_H_rew[idx_x, idx_y]: .2f}",
                                                    ha="center", va="center", color="w")
                txt[idx_x, idx_y, 2] = ax_H[1].text(idx_y, idx_x, f"{curr_H_trans[idx_x, idx_y]: .2f}",
                                                    ha="center", va="center", color="w")

        # plt.pause(.001)

        # 2) Looping through the memories
        for row_idx in range(1, self._agent_events.shape[0]):
            it = int(self._agent_events['iter'].iloc[row_idx])
            step = int(self._agent_events['step'].iloc[row_idx])

            # 2.a) If the agent's memory does not correspond to that of the environment, we quit
            # It is important to note here that during replay there's always a mismatch (hence if self > 0 we ignore)
            # and that if a reward is given, the agent is moved, so there's also a mismatch
            if step == 0 and self._agent_events['r'].iloc[row_idx] == 0 \
                    and self._agent_events['s_prime'].iloc[row_idx] != \
                    self._maze[int(self._events['agent_pos_x'].iloc[it]), int(self._events['agent_pos_y'].iloc[it])]:
                raise ValueError("mismatch between agent and environment memory")

            # 2.b) Else we have to see if we perform replay or not
            if step > 0:
                curr_replay = self.__replay_to_image__(curr_replay, row_idx)
            else:
                curr_replay = np.zeros(self._maze.shape)
            curr_maze = self.__status_to_image__(it)
            curr_Q = self.__event_to_img__(Q_vals.iloc[row_idx])
            curr_H_rew = self.__event_to_img__(H_rew_vals.iloc[row_idx])
            curr_H_trans = self.__event_to_img__(H_trans_vals.iloc[row_idx])

            # 2.c) Refresh txt
            for idx_x in range(txt.shape[0]):
                for idx_y in range(txt.shape[1]):
                    txt[idx_x, idx_y, 0].set_text(f"{curr_Q[idx_x, idx_y]: .2f}")
                    txt[idx_x, idx_y, 1].set_text(f"{curr_H_rew[idx_x, idx_y]: .2f}")
                    txt[idx_x, idx_y, 2].set_text(f"{curr_H_trans[idx_x, idx_y]: .2f}")

            # 2.d) Refresh plots
            axim_Q[0].set_data(curr_maze)
            axim_Q[1].set_data(curr_replay)
            axim_Q[2].set_data(curr_Q)
            axim_Q[1].autoscale()

            axim_H[0].set_data(curr_H_rew)
            axim_H[1].set_data(curr_H_trans)

            # 2.e) Stop
            fig_Q.canvas.flush_events()
            fig_H.canvas.flush_events()
            # plt.pause(.001)
