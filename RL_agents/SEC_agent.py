import numpy as np

'''
    Class implementing the Sequential Episodic Control (SEC) algorithm
'''

class SECagent(object):
    def __init__(self, action_space=4,
        pl=2, stm=50, ltm=500,
        sequential_bias=True, sequential_value=0.01,
        forget_mode="FIFO-PROP", forget_ratio=0.01,
        retrieval='default', similarity_threshold=0.1, k_neighbors=11,
        value_function='default', reward_decay=0.9, softmax=False, selection_mode='default',
        exploration_mode='default', exploration_steps=2500, epsilon=0.05,
        load_ltm=False):

        self.action_space = action_space
        self.state_length = pl

        self.stm_length = stm
        self.ltm_length = ltm

        self.sequential_bias = sequential_bias
        self.alpha_tr = sequential_value

        self.forgetting_mode = forget_mode
        self.forgetting_ratio = int(ltm*forget_ratio)
        self.memory_full = False

        self.retrieval_mode = retrieval
        self.similarity_threshold = similarity_threshold
        #self.k_neighbors = int(k_neighbors * self.action_space)
        self.k_neighbors = k_neighbors

        self.value_function = value_function
        self.tau_decay = reward_decay
        self.softmax = softmax
        self.selection_mode = selection_mode

        self.action = 0

        self.STM_states = np.zeros((self.stm_length, self.state_length))
        self.STM_actions = np.zeros((self.stm_length))

        self.LTM_states = np.empty((0, self.stm_length, self.state_length))
        self.LTM_actions = np.empty((0, self.stm_length))
        self.LTM_rewards = np.empty((0), float)
        self.LTM_lrus = np.empty((0), float)
        self.t = 0

        self.tr = np.empty((0, self.stm_length))
        #self.last_actions_indx = np.empty((0, self.stm_length), dtype=bool)
        self.selected_actions_indx = np.empty((0, self.stm_length), dtype=bool)

        self.entropy = 0.

        self.steps = 0
        self.exploration_mode = exploration_mode
        self.exploration_steps = exploration_steps # full random exploration time
        self.epsilon = epsilon

        self.print_variables()
        if load_ltm: self.load_LTM()

    def print_variables(self):
        print("STM length: ", self.stm_length)
        print("LTM length: ", self.ltm_length)
        print("Sequential bias: ", self.sequential_bias)
        print("Sequential value: ", self.alpha_tr)
        print("Forgetting mode: ", self.forgetting_mode)
        print("Forgetting ratio: ", self.forgetting_ratio)
        print("Retrieval mode: ", self.retrieval_mode)
        if self.retrieval_mode == 'default': print('Similarity threshold: ', self.similarity_threshold)
        if self.retrieval_mode == 'k_neighbors': print('K neighbors: ', self.k_neighbors)
        print('Value function mode: ', self.value_function)
        print('Reward decay: ', self.tau_decay)
        print('Softmax mode: ', self.softmax)
        print('Selection mode: ', self.selection_mode)
        print('Exploration mode: ', self.exploration_mode)
        print('Exploration steps: ', self.exploration_steps)
        print('Epsilon: ', self.epsilon)

    def choose_action(self, state):

        # For Atari games: choose exploration mode
        if self.exploration_mode == 'default':
            action, q = self.default_step(state)
        if self.exploration_mode == 'greedy':
            action, q = self.action_selection(state)
        if self.exploration_mode == 'epsilon':
            action, q = self.epsilon_step(state)
            #print('state is: ', state)
        if self.exploration_mode == 'epsilon_decay':
            action, q = self.epsilon_step(state)
            self.update_epsilon()

        # MEMORY UPDATE PHASE 1
        #self.update_STM(sa_couplet = [state, action])
        #self.update_sequential_bias()

        return action, q

    def update_epsilon(self):
        if self.epsilon > 0.05: #R
            self.epsilon -= (0.9/self.exploration_steps)

    def default_step(self, state):
        self.steps += 1
        # For Atari games: Chose CL action after a minimum number of exploration steps have been taken
        action, q = self.action_selection(state)
        if self.steps < self.exploration_steps:
            action = np.random.choice(a=self.action_space)
        return action, q

    def epsilon_step(self, state):
        # For Atari games: Follow an epsilon-greedy policy
        action, q = self.action_selection(state)
        if (np.random.random() < self.epsilon):
            action = np.random.choice(a=self.action_space)
        return action, q

    def action_selection(self, state):
        # get updated policy for a given state
        q = self.estimate_return(state)
        #print('Q: ', q)

        if self.selection_mode == 'default':
            # SEC DEFAULT: SAMPLE FROM WEIGHTED PROBABILITY
            self.action = np.random.choice(np.arange(q.shape[0]), p=q)

        if self.selection_mode == 'argmax':
            # RL STANDARD: ARGMAX
            self.action = np.argmax(q)

        q_action = q[self.action]

        if isinstance(self.action_space, list):
            self.action = [int(self.action/self.action_space[1]), self.action % self.action_space[1]]

        return self.action, q_action

    def estimate_return(self, state):
        # get the state-action value based on the memories stored in the LTM
        q = np.ones(self.action_space) / self.action_space

        if len(self.LTM_rewards) > 0:

            self.t += 1

            bias = 1
            if self.sequential_bias:
                bias = np.array(self.tr)

            if self.retrieval_mode == 'default':
                collectors_distance = np.sum(np.abs(state - self.LTM_states), axis=2)
                collectors_rel_similarity = 1 / (1 + collectors_distance)
                collectors = collectors_rel_similarity * bias
                collectors = collectors / collectors.max()
                self.selected_actions_indx = (collectors > self.similarity_threshold)

            if self.retrieval_mode == 'k_neighbors':
                collectors_distance = np.sum(np.abs(state - self.LTM_states), axis=2)
                collectors_rel_similarity = 1 - (collectors_distance / collectors_distance.max())
                collectors = collectors_rel_similarity * bias
                indices_selected = np.unravel_index(np.argsort(-collectors, axis=None)[:self.k_neighbors], collectors.shape)
                self.selected_actions_indx = np.zeros_like(collectors, dtype=bool)
                self.selected_actions_indx[indices_selected] = True

            if np.any(self.selected_actions_indx):
                actions = self.LTM_actions[self.selected_actions_indx]
                rewards = self.LTM_rewards[(np.nonzero(self.selected_actions_indx)[0])]
                distances = (self.stm_length - np.nonzero(self.selected_actions_indx)[1]) / self.stm_length
                collectors = collectors[self.selected_actions_indx]
                self.LTM_lrus[(np.where(self.selected_actions_indx.any(axis=1))[0])] = self.t

                q = self.get_policy(actions, collectors, rewards, distances)

        return q

    def get_policy(self, actions, collectors, rewards, distances):
        # map each selected action-vector into a matrix of N dimensions where N are the dimensions of the action space
        values = np.zeros((len(actions), self.action_space))
        if self.value_function == 'default':
            #print('COMPUTING ACTIONS CLASSIC SEC...')
            values[np.arange(len(actions)), actions[:].astype(int)] = collectors*(rewards*np.exp(-distances/self.tau_decay))
        if self.value_function == 'noGi':
            #print('COMPUTING ACTIONS WITHOUT SIMILARITY...')
            values[np.arange(len(actions)), actions[:].astype(int)] = rewards*np.exp(-distances/self.tau_decay)
        if self.value_function == 'noDist':
            #print('COMPUTING ACTIONS WITHOUT DISTANCE...')
            values[np.arange(len(actions)), actions[:].astype(int)] = collectors*rewards
        if self.value_function == 'noRR':
            #print('COMPUTING ACTIONS WITHOUT REWARD...')
            values[np.arange(len(actions)), actions[:].astype(int)] = collectors*np.exp(-distances/self.tau_decay)
        if self.value_function == 'soloGi':
            #print('COMPUTING ACTIONS WITH ONLY SIMILARTY...')
            values[np.arange(len(actions)), actions[:].astype(int)] = collectors
        if self.value_function == 'soloDist':
            #print('COMPUTING ACTIONS WITH ONLY DISTANCE...')
            values[np.arange(len(actions)), actions[:].astype(int)] = np.exp(-distances/self.tau_decay)
        if self.value_function == 'soloRR':
            #print('COMPUTING ACTIONS WITH ONLY RELATIVE REWARD...')
            values[np.arange(len(actions)), actions[:].astype(int)] = rewards

        if self.value_function == 'amax':
            #print('COMPUTING ACTIONS CLASSIC SEC...')
            values[np.arange(len(actions)), actions[:].astype(int)] = collectors*(rewards*np.exp(-distances/self.tau_decay))
            q = np.amax(values, axis=0) # get the best action value for each action from retrieved memories
        else:
            q = np.sum(values, axis=0) # aggregate action values from retrieved memories

        q = np.ravel(q)
        #print ("Pre POLICY: ", q)

        if self.softmax:
            # ACCURATE: normalization process of the value table.
            #q = np.exp(q) / np.exp(q).sum()  #-- sofmax function unstable for large numbers
            exp_q = np.exp(q - np.max(q))
            q = exp_q / exp_q.sum()
        else:
            # UNACCURATE (DEFAULT): proportion of being selected based on the action's relative reward based on the stored experiences
            q = q + np.abs(q.min())+1 # NEW - TO AVOID PROBLEMS WITH NEGATIVE REWARDS IF NOT USING SOFTMAX
            q = q/q.sum() # proportion of being selected based on the action's relative reward based on the stored experiences
            q = q.flatten()

        self.entropy = self.get_entropy(q)

        return q

    def get_entropy(self, policy):
        # Entropy of the prob distr for policy stability. (The sum of the % distribution multiplied by the logarithm -in base 2- of p)
        #q = np.ravel(policy)
        #print ("Post POLICY: ", policy)
        #print ("PROBS SUM: ", np.sum(policy))
        #exp_q = np.exp(policy - np.max(policy))
        #q = exp_q / exp_q.sum()
        entropy = np.sum(-policy * np.log2(policy + 1e-12))  # avoid log(0) by adding a small constant
        return entropy

    def update_STM(self, sa_couplet=[]):
        # Update STM buffer with the new state-action couplet (FIFO).
        self.STM_states = np.roll(self.STM_states, -1, axis=0)
        self.STM_actions = np.roll(self.STM_actions, -1, axis=0)
        self.STM_states[-1] = np.array(sa_couplet[0])
        self.STM_actions[-1] = sa_couplet[1]

    def update_sequential_bias(self):
        # Update trigger values.
        self.last_actions_indx = np.copy(self.selected_actions_indx) # Updates the last action indexes with the current actions indexes.

        if self.sequential_bias:
            # for an alpha_tr value of 0.1, the decay lasts 50 rounds to fall to 1 again
            if len(self.tr) > 0:
                #self.tr = (self.tr * (1. - self.alpha_tr)) + self.alpha_tr  # trigger values decay by default
                #self.tr = np.clip(self.tr, 1.0, None)   # all trigger values below 1 are clipped to 1
                self.tr = np.clip(self.tr * (1.0 - self.alpha_tr*5) + self.alpha_tr*5, 1.0, None)  # combine decay and clipping
                self.tr[self.last_actions_indx] = 1.0    # reset trigger values of previously selected actions
                tr_change_indx = np.roll(self.last_actions_indx, 1, axis=1) # shift the matrix one step to the right
                tr_change_indx[:, 0] = False  # set the first value of each sequence to False
                self.tr[tr_change_indx] += self.alpha_tr    # increase trigger values of newly selected actions
                #print('trigger values', self.tr)

    def reset_memory(self):
        # MEMORY RESET when finishing an episode
        self.reset_STM()
        self.reset_sequential_bias()

    def reset_sequential_bias(self):
        # Reset trigger values when beggining a new episode
        if len(self.tr) > 0:
            self.tr = np.ones_like(self.tr)

    def reset_STM(self):
        # Reset STM when beggining a new episode
        self.STM_states = np.zeros((self.stm_length, self.state_length))
        self.STM_actions = np.zeros((self.stm_length, 1))

    def update_LTM(self, reward=0):
        # MEMORY UPDATE PHASE 2
        # Update LTM based on current reward

        # Verify space of LTM
        self.check_LTM_space()

        reward_float = round(float(reward), 2)
        #print('REWARD: ', reward)

        # Update LTM if reached goal state and still have free space in LTM.
        if reward_float != 0:
            self.t += 1
            self.LTM_states = np.vstack((self.LTM_states, self.STM_states.reshape(1, self.stm_length, self.state_length)))
            self.LTM_actions = np.vstack((self.LTM_actions, self.STM_actions.reshape(1, self.stm_length)))
            self.LTM_rewards = np.append(self.LTM_rewards, reward_float)
            self.LTM_lrus = np.append(self.LTM_lrus, self.t)
            self.tr = np.vstack((self.tr, np.ones((self.stm_length))))
            self.selected_actions_indx = np.vstack((self.selected_actions_indx, np.zeros((self.stm_length), dtype=bool)))
            #self.last_actions_indx = np.vstack((self.last_actions_indx, np.zeros((self.stm_length), dtype=bool)))

    def check_LTM_space(self):
        #print("len rewards ",  len(self.LTM_rewards))
        if len(self.LTM_rewards) >= self.ltm_length:
            self.memory_full = True
            if self.forgetting_mode != "NONE":
                self.forget_LTM()

    def forget_LTM(self):
        #print("FORGETTING")
        if self.forgetting_mode == "FIFO-SING":
            idx_to_remove = 0
            # FIFO (Singular) - remove the oldest memory
        elif self.forgetting_mode == "FIFO-PROP":
            idx_to_remove = np.arange(self.forgetting_ratio)
            # FIFO (Proportional) - remove the oldest memories
        elif self.forgetting_mode == "RWD-SING":
            # RWD-SING (Singular) - remove the least valuable memory
            idx_to_remove = np.argmin(self.LTM_rewards)
        elif self.forgetting_mode == "RWD-PROP":
            # RWD-PROP (Proportional) - remove the least valuable memories
            idx_to_remove = np.argpartition(self.LTM_rewards, self.forgetting_ratio)[:self.forgetting_ratio]
        elif self.forgetting_mode == "LRU-SING":
            # LRU-SING (Singular) - remove the oldest and least useful memory
            idx_to_remove = np.argmin(self.LTM_lrus)
        elif self.forgetting_mode == "LRU-PROP":
            # LRU-PROP (Proportional) - remove the oldest and least useful memories
            idx_to_remove = np.argpartition(self.LTM_lrus, self.forgetting_ratio)[:self.forgetting_ratio]
        elif self.forgetting_mode == "TR-SING":
            # TR-SING (Singular) - remove the least triggered memory
            sequence_trs = np.sum(self.tr, axis=1)
            idx_to_remove = np.argmin(sequence_trs)
        elif self.forgetting_mode == "TR-PROP":
            # TR-PROP (Proportional) - remove the least triggered memories
            sequence_trs = np.sum(self.tr, axis=1)
            idx_to_remove = np.argpartition(sequence_trs, self.forgetting_ratio)[:self.forgetting_ratio]
        elif self.forgetting_mode == "TR-PROB":
            # TR-PROB (Probabilistic) - remove a random memory with a probability proportional to its usefulness
            p = 1 - (self.tr / np.sum(self.tr))
            p = p / np.sum(p)
            idx_to_remove = np.random.choice(range(len(self.tr)), p=p)
        self.remove_elements(idx_to_remove)

    def remove_elements(self, idx):
        self.LTM_states = np.delete(self.LTM_states, idx, axis=0)
        self.LTM_actions = np.delete(self.LTM_actions, idx, axis=0)
        self.LTM_rewards = np.delete(self.LTM_rewards, idx, axis=0)
        self.LTM_lrus = np.delete(self.LTM_lrus, idx, axis=0)
        self.tr = np.delete(self.tr, idx, axis=0)
        self.selected_actions_indx = np.delete(self.selected_actions_indx, idx, axis=0)
        #print("FORGOTTEN")

    def get_LTM_length(self):
        # In number of sequences
        ltm_len =  len(self.LTM_rewards)
        return ltm_len

    def get_memory_length(self):
        # In single memory units
        memories =  len(self.LTM_rewards) * self.stm_length
        return memories
