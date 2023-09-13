import numpy as np
from typing import Tuple
import pandas as pd
import pickle
import os
from math import *

# If we don't know the state space in advance, adding to _events memory dataframe will produce a performance warning due
# to "fragmentation". We, however, cannot be bothered at the moment.
from warnings import simplefilter
simplefilter(action="ignore", category=pd.errors.PerformanceWarning)


def entropy(proba: np.ndarray) -> float:
    """
    Computes the entropy of a vector of probabilities
    :param proba: said vector of probabilities
    :return: the corresponding entropy
    """
    if proba.sum() < 1-np.finfo(np.float32).eps or 1+np.finfo(np.float32).eps < proba.sum():  # Considering precision
        raise ValueError("Probabilities of outcomes must sum to 1.")
    proba = proba[proba != 0]
    return sum([-1 * p * log2(p) for p in proba])


class RLagent:
    """
    The agent class, capable of storing the metadata describing its model and the learned Q values. Furthermore, it will
    be capable of choosing an action based on different criteria, updating its Q values, and remembering and replaying
    its memory.
    """

    def __init__(self, act_num: int, max_rew: int, model_type: str, gamma: float, decision_rule: str, **kwargs):
        """
        Constructor for the basic instance of a Reinforcement Learning Agent.
        Exceptions: ValueError, if the str parameters are invalid
        :param act_num: the number of available actions
        :param max_rew: the maximal reward value
        :param model_type: temporal difference ['TD'], value iteration ['VI'] or policy iteration ['PI']
        :param gamma: discounting factor [float]
        :param kwargs: parameters regarding the decision rule, the replay and the epistemic rewards
            Regarding the model:
                beta: for softmax decisions (non-optional) [float]
                epsilon: for epsilon-greedy decisions (non-optional) [float]
                state_num: number of states in the environment [int] or None for unknown environment
                    curr_state: if the state_num is None, we need to know the starting state [int]
            Regarding the replay:
                replay_type: "forward", "backward", "priority", "trsam", "bidir" (optional) [str]
                    su_event: in the memory buffer, does a state-action couple constitute an event [True] or only a
                        state [False] (required, unless replay_type is "trsam")
                    replay_thresh: the replay threshold (minimum NORMALIZED error, optional) [float]
                    max_replay: the number of replay steps we'll take (optional) [int]
            Regarding the epistemic values:
                epist_decision: what epistemic value contributes to decision-making ("rew" for reward uncertainty,
                    "trans" for transition uncertainty, "both" for both, optional)
                epist_replay: IF REPLAY IS PRIORITY OR BIDIR, what epistemic value contributes to replay ("rew",
                    "trans" or "both", optional)
                    rew_weight: if "rew" or "both", what is the weight of the reward-related uncertainty-based epistemic
                        reward (required) [float between 0 and 1, weight of Q values is 1-rew_weight-trans_weight]
                    trans_weight: if "trans" or "both", what is the weight of the transition-related uncertainty-based
                        epistemic reward (required) [float between 0 and 1, weight of Q values is
                        1-rew_weight-trans_weight]
                    eta: what is the learning rate for epistemic rewards (required) [float]
                    replay_update: what epistemic values do we update during replay ("rew", "trans", "both", optional)
        """

        if model_type not in ["TD", "VI", "PI"]:
            raise ValueError("Model type '" + model_type + "' is not supported by the agent.")
        if decision_rule not in ["softmax", "max", "epsilon"]:
            raise ValueError("Decision rule '" + decision_rule + "' does not exist")

        # The parameters of the environment
        state_num = kwargs.get('state_num', None)
        if state_num is not None:
            self._nS = state_num  # number of possible states
        else:
            self._nS = 1  # We might only know a single state of the environment (the one we're in)
            curr_state = kwargs.get('curr_state', None)
            if curr_state is None:
                raise ValueError('In case of an environment with an unknown state space, '
                                 'the agent has to know the starting state')
            self._states = np.array([curr_state])  # an array of sates. The idx of each state is my state label
        self._nU = act_num  # max number of possible actions per state
        self._maxrew = max_rew  # max value of the achievable reward

        # The parameters of the agent
        self._gamma = gamma
        self._model_type = model_type
        self._decision_rule = decision_rule
        if self._decision_rule == "softmax":
            self._beta = kwargs.get("beta", None)
            if self._beta is None:
                raise ValueError("Decision rule 'softmax' requires temperature parameter 'beta'")
        elif self._decision_rule == "epsilon":
            self._epsilon = kwargs.get("epsilon", None)
            if self._epsilon is None:
                raise ValueError("Decision rule 'epsilon' requires exploration parameter 'epsilon'")

        # The agent's understanding of the environment
        self._Q = np.zeros((self._nS, self._nU))  # Q-values
        self._pi = np.random.randint(self._nU, size=self._nS)  # policy (what action to choose in each state)

        # On the replay
        self._replay_type = kwargs.get("replay_type", None)
        if self._replay_type not in [None, "forward", "backward", "priority", "trsam", "bidir"]:
            raise ValueError(f"Replay type {self._replay_type} is not a valid value.")
        if self._replay_type is not None:
            # Should I consider an event to be described by a state-action couple (True), or only by the state (False)
            self._su_event = kwargs.get("su_event", None)
            if self._replay_type != "trsam" and self._su_event is None:
                raise ValueError(f"In case of replay of type {self._replay_type}, su_event has to be specified."
                                 f"[True/False]")
            self._replay_thresh = kwargs.get("replay_thresh", 0)
            if self._replay_thresh < 0:
                raise ValueError('Replay threshold needs to be non-negative.')
            self._max_replay = kwargs.get("max_replay", None)
            if self._replay_thresh == 0 and self._max_replay is None:
                raise ValueError("Either the replay threshold or the maximum number of replay steps "
                                 "needs to be specified.")
            self._add_predecessors = None  # Only updated in MB agent
            self._trsam_forbidden_walls = None  # Only updated in MB agent
            # Memory: state (s), action (u), new state (s_prime), reward (r), surprise (delta)
            # formatted like a table, the first element is the one replayed
            # we need it except for trsam which will generate its own states to replay
            if self._replay_type != "trsam":
                if self._max_replay is not None:
                    self._memory_buff = np.zeros((self._max_replay, 5), dtype=float)
                    # Don't store more than what we can truly replay
                else:
                    self._memory_buff = np.zeros((1, 5), dtype=float)
                    # This one can be extended infinitely

        # For the epistemic rewards
        self._epist_decision = kwargs.get('epist_decision', None)
        if self._epist_decision not in [None, "rew", "trans", "both"]:
            raise ValueError("Field 'epist_decision' has to be of value 'rew', 'trans' or 'both'.")
        self._epist_replay = None
        if self._replay_type in ["priority", "bidir"]:
            self._epist_replay = kwargs.get('epist_replay', None)
            if self._epist_replay not in [None, "rew", "trans", "both"]:
                raise ValueError("Field 'epist_replay' has to be of value 'rew', 'trans' or 'both'.")
        if self._epist_decision is not None or self._epist_replay is not None:
            self._eta = kwargs.get('eta', None)  # Learning rate
            if self._eta is None:
                raise ValueError("Defining a learning rate for the epistemic reward functions is required.")
            self._replay_update = kwargs.get('replay_update', None)  # what are we updating during replay (if anything)
            if self._replay_update not in [None, "rew", "trans", "both"]:
                raise ValueError("Field 'replay_update' has to be of value 'rew', 'trans' or 'both'.")
        if self._epist_decision in ["rew", "both"] or self._epist_replay in ["rew", "both"]:
            self._rew_weight = kwargs.get('rew_weight', None)
            if self._rew_weight is None:
                raise ValueError("Reward-related uncertainty-based epistemic reward weight must be defined in advance.")
            # Proba of getting a reward in state s taking action u (init: 50%)
            self._rew_prob = np.ones((self._nS, self._nU)) * 0.5
        if self._epist_decision in ["trans", "both"] or self._epist_replay in ["trans", "both"]:
            self._trans_weight = kwargs.get('rew_weight', None)
            if self._trans_weight is None:
                raise ValueError(
                    "Transition-related uncertainty-based epistemic reward weight must be defined in advance.")
            # probability of getting to a state s' from state s with action u
            self._trans_prob = np.ones((self._nS, self._nU, self._nS)) / self._nS
            self._num_visit = np.zeros((self._nS, self._nU))

        # Regarding the dataframe of all events that happened during the lifetime of the agent
        self._save_agent = False  # Nothing is saved unless saving is specifically toggled
        self._events = None  # This will be the event memory if needed
        return

    # Methods related to the inner workings of the agent
    def __translate_s__(self, s: int) -> int | None:
        """
        Translates s into labels recognized by the agent (in case of automatically increasing state space)
        :param s: the state as detected from the environment
        :return: the state label recognized by the agent
        """
        if s is None:
            return s
        try:
            return np.argwhere(self._states == s)[0, 0]
        except AttributeError:
            return s

    def __su_entropy__(self, s: int, u: int, mode: str) -> float | None:
        """
        Computes the entropy under the specified circumstances
        :param s: The state
        :param u: The action
        :param mode: Do we compute reward-uncertainty ['rew'], or transition-uncertainty ['trans'] related entropy?
        :return: the entropy. If we requested a type of entropy the agent does not keep track of (e.g. 'rew' while the
            agent does not even have the field _rew_prob, the return value is None)
        """
        if mode not in ['rew', 'trans']:
            raise ValueError('Only transition and reward entropy can be computed this way.')
        if mode == 'rew':
            if self._epist_decision in ["rew", "both"] or self._epist_replay in ["rew", "both"]:
                return entropy(np.array([self._rew_prob[s, u], 1 - self._rew_prob[s, u]]))
        if self._epist_decision in ["trans", "both"] or self._epist_replay in ["trans", "both"]:
            return entropy(np.array([self._trans_prob[s, u, idx_x] for idx_x in range(self._nS)]))
        return

    def __combine_Q_epist__(self, Q: float, **kwargs) -> float:
        """
        Combines the normalized Q and epistemic values based on the pre-defined ratio for state. This will only be used
        to assess the priority of certain updates, that is Q values will be stored and updated in the usual fashion
        :return: the combined value normalized between 0 and 1; or the combined delta value normalized between -1 and 1
        :param Q: the Q value [0, max_Q] or delta_Q value [-max_Q, max_Q] to be combined
        :param kwargs:
            H_rew: the reward-related uncertainty-based epistemic value [0, max_H_rew]
                (or the difference thereof [-max_H_rew, max_H_rew])
            H_trans: the transition-related uncertainty-based epistemic value [0, max_H_trans]
                (or the difference thereof [-max_H_trans, max_H_trans])
        """
        # Setting the weights
        if self._maxrew != 0:
            Q = Q / self._maxrew
        Q_weight = 1
        H_rew = kwargs.get('H_rew', None)
        if H_rew is not None:
            H_rew = H_rew / entropy(np.ones(2) / 2)
            Q_weight -= self._rew_weight
        H_trans = kwargs.get('H_trans', None)
        if H_trans is not None:
            if self._nS > 1:
                H_trans = H_trans / entropy(np.ones(self._nS) / self._nS)
                Q_weight -= self._trans_weight
            else:
                # In case the state space consists of a single state, we can consider that the entropy is maximal, as
                # the agent has not yet performed any form of exploration
                H_trans = 1

        # Computing the weighted sum
        Q_comb = Q_weight * Q
        if H_rew is not None:
            Q_comb += self._rew_weight * H_rew
        if H_trans is not None:
            Q_comb += self._trans_weight * H_trans
        return Q_comb

    def __store_in_memory__(self, s: int, u: int, s_prime: int, r: float, delta: float) -> None:
        """
        Takes an action and stores it in the memory buffer. The memory buffer is a numpy array, where each row is a new
        memory. The rows contain [state, action, new_state, reward, (combined)_Q_value_difference]
        During replay, it's always the first element in the array that gets replayed first. This means, that for forward
        replay, the top row will contain the oldest element, for backwards replay it will contain the newest memory, and
        for prioritized, it will contain the most surprising memory.
        The new memory can be added to the buffer, or if a copy of it already exists in the buffer, then it can be
        modified or deleted.
        :param s: current state idx
        :param u: taken action
        :param s_prime: arrival state
        :param r: gained reward
        :param delta: change in the (combined) normalized Q value
        :return: -
        """
        if self._replay_type == "trsam":
            return  # No need to store anything

        # 0) In certain cases, we will have to simply delete a memory entry. This will be decided here
        to_delete = False  # This variable is True, if this element might have to be simply deleted instead of updated
        # This can happen in prioritized sweeping (or bidir), when an element is already in the buffer, but its
        # newest instance carries less surprise than the threshold itself.
        if abs(delta) <= self._replay_thresh and self._replay_type in ["priority", "bidir"]:
            to_delete = True
        to_store = np.array([[s, u, s_prime, r, delta]])  # for simplicity's sake, this is our new row

        # 1) If the buffer is empty, and the element is not sub-threshold, just add the element
        empty_idx = np.where(np.all(self._memory_buff == 0, axis=1))[0]  # IDX of all empty rows
        if not empty_idx.size == 0 and empty_idx[0] == 0 and not to_delete:
            self._memory_buff[0, :] = to_store
            return
        else:
            # 2) If this element is already in the buffer, let's take the old copy out (we can later insert the new)
            # We can identify an event based on state only, or a state-action pair, decided by the su_event param
            memory_idx = None
            if self._su_event and np.any(
                    np.logical_and(self._memory_buff[:, 0] == s, self._memory_buff[:, 1] == u)):
                memory_idx = np.where(np.logical_and(self._memory_buff[:, 0] == s, self._memory_buff[:, 1] == u))[0]

            if not self._su_event and np.any(self._memory_buff[:, 0] == s):
                memory_idx = np.where(self._memory_buff[:, 0] == s)[0]

            if memory_idx is not None and memory_idx.size != 0:  # If we found a copy

                if to_delete and self._max_replay is None:
                    # 2.a.1) if we have infinite capacity, and we want to remove the element for sure,
                    # this deletion should reduce the array size
                    self._memory_buff = np.delete(self._memory_buff, obj=memory_idx[0], axis=0)
                else:
                    # 2.a.2) otherwise just scrape this row and shift the elements so that the empty row is at the
                    # bottom of the array
                    self._memory_buff[memory_idx[0], :] = np.zeros((1, 5))  # delete the pre-existing copy
                    self._memory_buff[memory_idx[0]:, :] = np.roll(self._memory_buff[memory_idx[0]:, :], -1, axis=0)
                    # Now the bottom row is empty for sure

                if empty_idx.size != 0:
                    # If we deleted a row from a non-full buff, the IDX of the first empty row decreased
                    empty_idx -= 1
                elif self._max_replay is not None:
                    # If it was full but not infinite, now the last row is empty
                    empty_idx = np.array([self._memory_buff.shape[0] - 1])

            # 2.b) if the new instance is for deletion, whether we deleted its old copy or not, we're done here
            if to_delete:
                return

            # 3) Whether we removed the old copy or not, now we just insert the new element
            # 3.1) If full, but infinite capacity, then add a row
            if empty_idx.size == 0 and self._max_replay is None:
                self._memory_buff = np.append(self._memory_buff, np.zeros((1, 5)), axis=0)
                empty_idx = np.array([self._memory_buff.size[0] - 1])

            if self._replay_type in ["backward", "priority", "bidir"]:
                # 3.a.1) Find the appropriate insertion idx
                insertion_idx = 0  # backwards -- put it on the front
                if self._replay_type in ["priority", "bidir"]:
                    insertion_idx = np.where(abs(self._memory_buff[:, -1]) < abs(delta))[0]
                    if insertion_idx.size == 0:  # if full and all the stored elements are more important
                        return
                    else:
                        insertion_idx = insertion_idx[0]

                # 3.a.2) Shift everything down below the insertion idx, then overwrite
                self._memory_buff[insertion_idx:, :] = np.roll(self._memory_buff[insertion_idx:, :], 1, axis=0)
                self._memory_buff[insertion_idx, :] = to_store
                return

            elif self._replay_type == "forward":
                insertion_idx = empty_idx  # IDX of first empty row
                if insertion_idx.size == 0:
                    # 3.b) If full, we'll have to insert to the bottom of the table. Remember that if we have infinite
                    # capacity, we already inserted an empty row at the bottom, so this branch does not execute
                    # 3.b.1) Thus: since we are full and finite, shift up and overwrite the last element
                    self._memory_buff = np.roll(self._memory_buff, -1, axis=0)
                    insertion_idx = np.array([-1])
                # 3.b.2) If not full, or already shifted up. just insert the element to the end
                self._memory_buff[insertion_idx[0], :] = to_store
                return

    def __epist_update__(self, s: int, u: int, **kwargs) -> float | np.ndarray | Tuple[float, np.ndarray] | \
                                                            None | Tuple[None, None]:
        """
        A function that can serve two different purposes. It can either estimate and return the updated epistemic values
        of a state (used for finding proper predecessors) or it can actually perform the update, in this case, returning
        nothing.
        :param s: The state on which the update shall be performed
        :param u: The action on which the update shall be performed
        :param kwargs:
            r: The received reward in (s, u). If specified, the reward-related uncertainty update will be computed
                (optional) [float]
            s_prime: The arriving state after (s, u). If specified, the transition-related uncertainty update will be
                computed (optional) [int]
            update: Should we actually update the reward- and/or transition-related uncertainty [True] or should we
                return the updated values without performing the updates [False]
        :return: None if update; the updated reward probability [float] and/or the updated transition probabilities
            [np.ndarray] if not update. In the latter case no actual update will take place
        """
        r = kwargs.get('r', None)
        s_prime = kwargs.get('s_prime', None)
        update = kwargs.get('update', True)
        rew_prob, trans_prob = None, None
        num_visit, trans_prob = None, None
        if self._epist_decision in ["rew", "both"] or self._epist_replay in ["rew", "both"]:
            rew_prob = self._rew_prob[s, u]
        else:
            r = None  # if the criterion is false we don't even have _rew_prob, so no reason to update
        if self._epist_decision in ["trans", "both"] or self._epist_replay in ["trans", "both"]:
            num_visit = self._num_visit[s, u]
            trans_prob = self._trans_prob[s, u, :]
        else:
            s_prime = None  # if the criterion is false we don't even have _trans_prob, so no reason to update

        # Computing updated reward probabilities
        if r is not None:
            r = float(r > 0)  # Let r be 0 or 1
            rew_prob = (1 - self._eta) * rew_prob + self._eta * r
            if not update and s_prime is None:
                return rew_prob

        # Computing updated transition probabilities
        if s_prime is not None:
            num_visits = num_visit + 1
            trans_prob = ((1 - 1 / num_visits) * trans_prob
                          + np.reshape(np.array(range(self._nS)) == s_prime, (1, 1, self._nS)) / num_visits)
            if not update and r is None:
                return trans_prob

        # If we wanted to estimate both
        if not update:
            return rew_prob, trans_prob

        # If we really want to update, and we have the required variables, update
        if r is not None:
            self._rew_prob[s, u] = rew_prob
        if s_prime is not None:
            self._num_visit[s, u] = num_visit
            self._trans_prob[s, u, :] = trans_prob
        return

    def __extend_state_space__(self, s_prime) -> None:
        """
        Upon encountering a never-before seen state, this function extends the state-space
        :param s_prime: the label of the new state
        :return:
        """
        self._states = np.append(self._states, np.array([s_prime]), axis=0)
        self._Q = np.append(self._Q, np.zeros((1, self._nU)), axis=0)

        try:  # If model-based
            self._N = np.append(self._N, np.zeros((1, self._nU)), axis=0)
            self._T = np.append(self._T, np.zeros((self._nS, self._nU, 1)), axis=2)  # Transition to new state
            self._T = np.append(self._T, np.ones((1, self._nU, self._nS + 1)) / (self._nS + 1),
                                axis=0)  # Tr from new state
            self._R = np.append(self._R, np.zeros((1, self._nU)), axis=0)
        except AttributeError:
            pass

        if self._epist_decision in ["trans", "both"] or self._epist_replay in ["trans", "both"]:
            self._num_visit = np.append(self._num_visit, np.zeros((1, self._nU)), axis=0)
            self._trans_prob = np.append(self._trans_prob, np.zeros((self._nS, self._nU, 1)), axis=2)
            self._trans_prob = np.append(self._trans_prob, np.ones((1, self._nU, self._nS + 1)) / (self._nS + 1), axis=0)

        if self._epist_decision in ["rew", "both"] or self._epist_replay in ["rew", "both"]:
            self._rew_prob = np.append(self._rew_prob, np.ones((1, self._nU)) * 0.5, axis=0)

        self._nS += 1

        self.toggle_save(save_on=self._save_agent)  # To extend the saved dataframe too
        return

    def __find_good_actions__(self, s: int, **kwargs) -> np.ndarray:
        """
        Placeholder for the MB agent finding good (or possible) actions based on the model.
        :param s: Current state
        :param kwargs:
            prev_s: Previous state we don't want ot go back to
        :return:
        """
        pass

    # Hidden methods that children classes will use
    def __temporal_difference_error__(self, s: int, u: int, s_prime: int, r: float) -> float:
        """
        Placeholder for the TDE algorithm of the MF class
        :param s: current state label
        :param u: chosen action
        :param s_prime: arrival state label
        :param r: received reward
        :return: prediction error
        """
        pass

    def __value_iteration__(self, s: int, u: int) -> Tuple[float, np.ndarray]:
        """
        Placeholder for the VI algorithm of the MB class
        :param s: current state
        :param u: action chosen
        :return: the difference in the Q value after update
        """
        pass

    def __policy_iteration__(self, s: int, u: int) -> Tuple[float, np.ndarray]:
        """
        Placeholder for the PI algorithm of the MB agent.
        :param s: current state
        :param u: chosen action
        :return: the difference in the Q value after update
        """
        pass

    def __trajectory_sampling__(self, s: int, **kwargs) -> None:
        """
        Placeholder for the trajectory sampling function of the model-based agent.
        :param s: state we start the simulation from
        :param kwargs:
            stop_loc: state(s) we terminate in [np.ndarray]
            steps: how many steps we are going to simulate [int]
        :return:
        """
        stop_loc = kwargs.get('stop_loc', None)
        steps = kwargs.get('steps', self._max_replay)
        return

    def __find_predecessors__(self, s: int, val_fun: np.ndarray) -> None:
        """
        Placeholder function, it is expanded in MBagent
        :param s: current state
        :param val_fun: value function by which we update (Q or V, depending the self._model_type)
        :return: -
        """
        pass

    # Hidden method concerning the saving
    def __save_step__(self, virtual: bool, **kwargs) -> None:
        """
        Saves the current state of the maze by adding a row to the _events memory.
        :param virtual: is this a virtual step [True] or a real one [False]
        :param kwargs:
            s: last state
            u: last action
            s_prime: current state
            r: last reward
            delta: (combined) delta Q of the learning step
        :return:
        """
        if not self._save_agent:
            return

        # 1) Which step are we at
        it, step = 0, 0
        if self._events.shape[0] > 0:
            if not virtual:  # If real, it's a new iteration
                it = self._events['iter'].iloc[-1] + 1
            else:  # else it's the same iteration but a new step
                it = self._events['iter'].iloc[-1]
                step = self._events['step'].iloc[-1] + 1

        # 2) Format the event to store
        # iter, step, s, u, s', r, (combined)deltaQ, Q values, (H_r values, H_t values)
        # for the latter three it's ordered Q(s1, u1), Q(s1, u2), ..., Q(s1, uk), Q(s2, u1), ..., Q(sn, uk)
        event = np.zeros(self._events.shape[1])  # None's at the place of s, u, s_prime, r mean no step was taken
        event[0] = it
        event[1] = step
        s = kwargs.get('s', None)
        if s is not None:
            try:
                event[2] = self._states[s]
            except AttributeError:
                event[2] = s
        else:
            event[2] = None
        event[3] = kwargs.get('u', None)
        s_prime = kwargs.get('s_prime', None)
        if s_prime is not None:
            try:
                event[4] = self._states[s_prime]
            except AttributeError:
                event[4] = s_prime
        else:
            event[4] = None
        event[5] = kwargs.get('r', None)
        event[6] = kwargs.get('delta', None)

        # Now the Q and H values
        event[7:7 + self._nS * self._nU] = np.reshape(self._Q, (self._nS * self._nU,), order='C')
        last_idx = 7 + self._nS * self._nU
        if self._epist_decision in ["rew", "both"] or self._epist_replay in ["rew", "both"]:
            event[last_idx:last_idx + self._nS * self._nU] = np.array([self.__su_entropy__(s_idx, u_idx, 'rew')
                                                                       for s_idx in range(self._nS)
                                                                       for u_idx in range(self._nU)])
            last_idx += self._nS * self._nU
        if self._epist_decision in ["trans", "both"] or self._epist_replay in ["trans", "both"]:
            event[last_idx:last_idx + self._nS * self._nU] = np.array([self.__su_entropy__(s_idx, u_idx, 'trans')
                                                                       for s_idx in range(self._nS)
                                                                       for u_idx in range(self._nU)])

        # 3) Add it to the table
        self._events = pd.concat([self._events, pd.DataFrame([event], columns=self._events.columns)])
        return

    # Methods used to instruct the agent
    def choose_action(self, s: int, u_poss: np.ndarray, **kwargs) -> Tuple[int, float]:
        """
        Chooses actions from the available ones from a predefined state and the set of available actions observed from
        env. The action choice will depend on the decision rule. We might use a softmax function, a greedy choice by Q,
        or an epsilon greedy one.
        :param s: state from which we want to take a step
        :param u_poss: array of possible actions, given by the env
        :param kwargs:
            virtual: is this a virtual step (True) or a real one (False, default)
        :return: chosen action and the corresponding Q value
        """
        # Let's see what epistemic values we will have to combine (is this a real decision, or a virtual replay?)
        virtual = kwargs.get('virtual', False)
        combine = self._epist_decision
        if virtual:
            combine = self._epist_replay
        else:  # If s comes from the environment
            s = self.__translate_s__(s)

        # Let's make the decision:
        # 1) if epsilon greedy, and we explore
        if self._decision_rule == "epsilon":
            # Simplest case, we choose randomly
            if np.random.uniform(0, 1, 1) <= self._epsilon:
                u = np.random.choice(u_poss)
                return int(u)

        # 2) For the other methods combine all the potential constituents
        Q_poss = np.array([self._Q[s, idx_u] for idx_u in u_poss])
        H_rew_poss, H_trans_poss = np.array([None] * len(u_poss)), np.array([None] * len(u_poss))
        if combine in ['rew', 'both']:
            H_rew_poss = np.array([self.__su_entropy__(s, idx_u, 'rew') for idx_u in u_poss])
        if combine in ['trans', 'both']:
            H_trans_poss = np.array([self.__su_entropy__(s, idx_u, 'trans') for idx_u in u_poss])
        comb_Q_poss = np.array([self.__combine_Q_epist__(Q_poss[idx], H_rew=H_rew_poss[idx],
                                                         H_trans=H_trans_poss[idx]) for idx in range(len(u_poss))])
        # comb_Q_poss is between 0 and 1

        # 3) If we choose to put these combined values through a softmax
        if self._decision_rule == "softmax":
            p_poss = np.exp(self._beta * comb_Q_poss) / np.sum(np.exp(self._beta * comb_Q_poss))
            u = np.random.choice(u_poss, p=p_poss)
            return int(u)

        # 4) If we choose the maximum (either due to greedy or epsilon greedy policies)
        u_poss = u_poss[comb_Q_poss == max(comb_Q_poss)]
        u = np.random.choice(u_poss)
        return int(u), self._Q[s, u]

    def learnQvalues(self, s: int, u: int, s_prime: int, r: float, **kwargs) -> float:
        """
        Overwrites the parent class's method. Uses TDE to actualy update the Q values
        :param s: current state label
        :param u: taken action label
        :param s_prime: arriving state label
        :param r: reward value
        :param kwargs:
            update_buffer: should I store this item in the memory buffer [True, default] or not [False] -- during
                forward and backward replay or trsam we don't want to update the memory buffer
            virtual: is this a virtual step [True] or a real one [False, default] -- will decide if we'll learn epist
                values from it, and *in case we update the buffer*, do we add predecessors to it
        :return: the (combined) delta Q
        """
        # Let's see what situation we're in (real step or virtual, do we update the buffer or not)
        update_buffer = kwargs.get('update_buffer', True)
        virtual = kwargs.get('virtual', False)
        combine = self._epist_replay  # it will only really be used to influence the memory_buffer

        if not virtual:  # If s comes from the environment
            try:
                if s_prime not in self._states:
                    self.__extend_state_space__(s_prime)
            except AttributeError:
                pass
            s, s_prime = self.__translate_s__(s), self.__translate_s__(s_prime)
            # Extend the state space if needed

        # 0) Handle the potential problem of out-of bounds Q-values (this is added for the case when a reward
        # appears. Since we don't want to make that state overrepresented, we have to delete the OG Q value.)
        if r > 0:
            for u_prime in range(self._nU):
                self._Q[
                    s_prime, u_prime] = 0  # If we set it to zero @ the reward, it will never be larger than max_rew

        # 1) learn Q val (each model will do its own thing)
        delta_Q, val_func = 0, np.array([])
        if self._model_type == 'TD':
            delta_Q = self.__temporal_difference_error__(s, u, s_prime, r)
        elif self._model_type == 'VI':
            delta_Q, val_func = self.__value_iteration__(s, u)
        elif self._model_type == "PI":
            delta_Q, val_func = self.__policy_iteration__(s, u)

        # 2) learn H val if it is a real step or if we want to learn from replay
        H_rew, H_trans = None, None
        if self._epist_decision in ["rew", "both"] or self._epist_replay in ["rew", "both"]:  # otherwise no _rew_proba
            H_rew = self.__su_entropy__(s, u, 'rew')
        if self._epist_decision in ["trans", "both"] or self._epist_replay in ["trans", "both"]:  # otherwise no _trans
            H_trans = self.__su_entropy__(s, u, 'trans')
        if not virtual or \
                ((self._epist_decision is not None or self._epist_replay is not None) and self._replay_update):
            self.__epist_update__(s, u, r=r, s_prime=s_prime)

        # 3) combine the Q and H values if needed
        delta_H_rew, delta_H_trans = None, None
        if combine in ['rew', 'both']:
            delta_H_rew = self.__su_entropy__(s, u, 'rew') - H_rew
        if combine in ['trans', 'both']:
            delta_H_trans = self.__su_entropy__(s, u, 'trans') - H_trans
        comb_delta_Q = self.__combine_Q_epist__(delta_Q, H_rew=delta_H_rew, H_trans=delta_H_trans)  # between -1 and 1

        # 4) Store if needed
        if update_buffer:
            self.__store_in_memory__(s, u, s_prime, r, comb_delta_Q)
            if self._replay_type in ['priority', 'bidir'] and \
                    ((not virtual and self._add_predecessors in ['act', 'both']) or
                     (virtual and self._add_predecessors in ['rep', 'both'])):
                self.__find_predecessors__(s, val_func)
        # Saving the step in a table
        self.__save_step__(virtual, s=s, u=u, s_prime=s_prime, r=r, delta=comb_delta_Q)

        return comb_delta_Q

    def memory_replay(self, **kwargs) -> None:
        """
        Performs the memory replay. It either goes through the memory buffer, and performs a learning step on the stored
        events, in an order predefined by self._replay_type; or calls trajectory_sampling to generate virtual experience
        :param kwargs:
            s: the label of the starting state, in case the replay is trajectory sampling [int]
        :return:
        """

        # 0) This function uses stored memories to replay. For simulating experience, we need to call trsam
        update_buffer = False  # We will most likely not update the memory buffer during replay
        if self._replay_type == 'trsam':
            s = kwargs.get('s', None)
            if s is None:
                raise ValueError('Trajectory sampling needs a starting state specified.')
            self.__trajectory_sampling__(s)
            return
        elif self._replay_type in ['priority', 'bidir']:
            # For bidir and priority it is essential to always update the buffer, as priorities change during replay
            update_buffer = True

        delta = 0  # how big the last (combined) Q value change was
        it = 0  # how many iterations of memory replay have e performed so far
        buffer_idx = 0  # which memory are we replaying (for priority and bidir it's always 0)
        stop_loc = np.array([])  # for bidirectional, we might want to collect the states in which we need to stop

        # 1.1) Iterate while we can
        while self._max_replay is None or it < self._max_replay:
            event = self._memory_buff[buffer_idx]  # buffer_idx == 0 for priority and bidir
            if self._replay_type in ["priority", "bidir"]:
                # 1.2.a) If priority/bidir, we'll check significance: if we're still significant (delta > threshold) we
                # take the first element, and perform the update on it. If it's an empty event, that means the buffer
                # is empty, and the replay is over
                if np.all(event == 0) or (it > 0 and abs(delta) < self._replay_thresh):
                    break
                delta = 0
            elif self._replay_type in ["forward", "backward"]:
                # 1.2.b) If forward or backward, we need to loop through the entire memory buffer (without changing it)
                # before we can tell whether the updates are significant or not. If we never encountered a single
                # above-threshold delta over the whole array, we stop. Otherwise, we just restart.
                if np.all(event == 0) or (it > 0 and buffer_idx == 0):
                    if abs(delta) < self._replay_thresh:
                        break
                    delta = 0
                    if buffer_idx != 0:
                        buffer_idx = 0
                        event = self._memory_buff[buffer_idx]

            # 1.3) We perform the replay, and we learn from it. The learning takes place on virtual experience,
            # but whether we update the buffer or not depend on whether we use priorities or not
            try:  # if MB we have a _R reward function
                # Based on whether we consider events as recollections of s, or (s, u), the rest will be filled by the
                # model
                s = int(event[0])
                u = int(event[1])
                if not self._su_event:
                    u_poss = self.__find_good_actions__(s)
                    u = self.choose_action(s, u_poss, virtual=True)
                s_prime = np.random.choice(list(range(self._nS)), p=self._T[s, u, :])
                delta_curr = self.learnQvalues(s=s, u=u, s_prime=s_prime,
                                               r=self._R[int(event[0]), int(event[1])],
                                               virtual=True, update_buffer=update_buffer)
            except AttributeError:  # if MF, we get an AttributeError, and we can try again
                delta_curr = self.learnQvalues(s=int(event[0]), u=int(event[1]), s_prime=int(event[2]),
                                               r=int(event[3]), virtual=True, update_buffer=update_buffer)

            # 1.4) Conclude by some final step
            if self._replay_type in ["priority", "bidir"]:
                delta = delta_curr
                if self._replay_type == 'bidir':
                    # We store the replayed state as a bidir stopping criterion
                    # (we only store s, not s_prime, as technically a decision from s_prime has not yet been replayed)
                    stop_loc = np.append(stop_loc, np.array([int(event[0])]), axis=0)
            elif self._replay_type in ["forward", "backward"]:
                if abs(delta_curr) > abs(delta):
                    delta = delta_curr
                buffer_idx += 1
                if buffer_idx >= self._memory_buff.shape[0]:
                    # No need to take care of arriving at an empty row, that's handled in 1.2.b)
                    buffer_idx = 0
            it += 1

        # 2) If bidir, let's run some simulations using the remaining steps
        if self._replay_type == 'bidir':
            s = kwargs.get('s', None)
            if s is None:
                raise ValueError('Bidirectional search needs a starting state specified.')
            self.__trajectory_sampling__(s, stop_loc=np.unique(stop_loc), steps=self._max_replay - it)
        return

    def agent_type(self) -> None | str:
        """
        Simple getter for the agent type
        :return: 'MB' or 'MF'
        """
        return

    def update_agent(self, max_rew: float) -> None:
        """
        Updates the agent's expectations about the reward, should a change take place in the environment.
        :param max_rew: the new max reward
        :return:
        """
        self._maxrew = max_rew  # max value of the achievable reward

    # All about saving
    def toggle_save(self, **kwargs) -> None:
        """
        Toggles save. If the agent was saving its status so far, it sops doing so. Otherwise, it begins to do so,
        by already storing a snapshot of the current state as well.
        Important to note that this function can also be called to extend the saved table in case a new state is
        encountered.
        :param kwargs:
            save_on: If instead of toggling, we want to make sure to turn it on [True] or off [False], we can
        :return:
        """
        save_on = kwargs.get('save_on', not self._save_agent)
        if save_on:
            try:
                # We need to know if we have 1 dataset (Q vals) 2 (Q and H) or 3 (Q and 2 types of H)
                datasets = 1 + int(self._epist_decision in ["rew", "both"] or self._epist_replay in ["rew", "both"]) + \
                            int(self._epist_decision in ["trans", "both"] or self._epist_replay in ["trans", "both"])
                if self._events.shape[1] != 7 + datasets * self._nS * self._nU:
                    # If we recently added a new state (that is we don't have 6 + Q val (+ H_rew + H_trans) values)
                    # IMPORTANT! We assume that a single new state was introduced and we extend immediately!
                    for u_idx in range(self._nU):
                        self._events.insert(6 + (self._nS-1) * self._nU + u_idx + 1, f'Q_{self._states[-1]}_{u_idx}',
                                            np.zeros(self._events.shape[0]))
                        if self._epist_decision in ["rew", "both"] or self._epist_replay in ["rew", "both"]:
                            self._events.insert(6 + 2 * ((self._nS-1) * self._nU + u_idx) + 2,
                                                f'H_rew_{self._states[-1]}_{u_idx}', np.zeros(self._events.shape[0]))
                            # We use zeros to show we have no clue
                        if self._epist_decision in ["trans", "both"] or self._epist_replay in ["trans", "both"]:
                            self._events[f'H_trans_{self._states[-1]}_{u_idx}'] = np.zeros(self._events.shape[0])
            except AttributeError:  # There is no such thing as _events
                poss_states = range(self._nS)
                try:
                    poss_states = self._states
                except AttributeError:
                    pass
                Q_names = [f'Q_{s_idx}_{u_idx}' for s_idx in poss_states for u_idx in range(self._nU)]
                H_rew_names, H_trans_names = [], []
                if self._epist_decision in ["rew", "both"] or self._epist_replay in ["rew", "both"]:
                    H_rew_names = [f'H_rew_{s_idx}_{u_idx}' for s_idx in poss_states for u_idx in range(self._nU)]
                if self._epist_decision in ["trans", "both"] or self._epist_replay in ["trans", "both"]:
                    H_trans_names = [f'H_trans_{s_idx}_{u_idx}'
                                     for s_idx in poss_states for u_idx in range(self._nU)]
                self._events = pd.DataFrame(columns=['iter', 'step', 's', 'u', 's_prime', 'r', 'deltaQ',
                                                     *Q_names, *H_rew_names, *H_trans_names])
            if not self._save_agent:
                self._save_agent = True
                self.__save_step__(True)  # If we just turned it on, we take a snapshot of the agent's current state
        else:
            self._save_agent = False

    def dump_agent(self, **kwargs) -> None:
        """
        Saves everything that we have stored into 2 different files: one for the agent, and one for the events.
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

        # 1) Save the whole agent
        file = open(f'{path}agent{label}.txt', 'wb')
        pickle.dump(self.__dict__, file, 2)
        file.close()

        # 2) Save the events
        try:
            self._events.to_csv(f'{path}agent{label}.csv', sep=',', index=False, encoding='utf-8')
        except AttributeError:
            print('Note: This agent does not store the transpired events, no .csv generated.')

    def load_agent(self, file_name: str, **kwargs):
        """
        Loads a previously saved agent
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


class MFagent(RLagent):
    """
    The model free agent class. It will know the TDE method, and it will overwrite learning with it.
    """

    def __init__(self, act_num: int, max_rew: int, model_type: str, gamma: float, decision_rule: str, alpha: float, **kwargs):
        """
        Constructor for the MF agent
        Exceptions: ValueError, if the str parameters are invalid
        :param act_num: the number of available actions
        :param max_rew: the maximal reward value
        :param model_type: temporal difference, value iteration or policy iteration
        :param gamma: discounting factor
        :param alpha: learning parameter of TD learning
        :param kwargs: parameters regarding the decision rule, the replay and the epistemic rewards
            Regarding the model:
                beta: for softmax decisions (non-optional) [float]
                epsilon: for epsilon-greedy decisions (non-optional) [float]
                state_num: number of states in the environment [int] or None for unknown environment
                    curr_state: if the state_num is None, we need to know the starting state [int]
            Regarding the replay:
                replay_type: "forward", "backward", "priority" (optional) [str]
                    su_event: in the memory buffer, does a state-action couple constitute an event [True] or only a
                        state [False] (required)
                    replay_thresh: the replay threshold (minimum NORMALIZED error, optional) [float]
                    max_replay: the number of replay steps we'll take (optional) [int]
            Regarding the epistemic values:
                epist_decision: what epistemic value contributes to decision-making ("rew" for reward uncertainty,
                    "trans" for transition uncertainty, "both" for both, optional)
                epist_replay: IF REPLAY IS PRIORITY, what epistemic value contributes to replay ("rew",
                    "trans" or "both", optional)
                    rew_weight: if "rew" or "both", what is the weight of the reward-related uncertainty-based epistemic
                        reward (required) [float between 0 and 1, weight of Q values is 1-rew_weight-trans_weight]
                    trans_weight: if "trans" or "both", what is the weight of the transition-related uncertainty-based
                        epistemic reward (required) [float between 0 and 1, weight of Q values is
                        1-rew_weight-trans_weight]
                    eta: what is the learning rate for epistemic rewards (required) [float]
                    replay_update: what epistemic values do we update during replay ("rew", "trans", "both", optional)
        """
        if model_type != "TD":
            raise ValueError("Model type '" + model_type + "' is not supported by the MF agent.")
        replay_type = kwargs.get('replay_type', None)
        if replay_type not in [None, "forward", "backward", "priority"]:
            raise ValueError("Replay type '" + replay_type + "' is not supported by the MF agent.")
        RLagent.__init__(self, act_num, max_rew, model_type, gamma, decision_rule,
                         beta=kwargs.get('beta', None),
                         epsilon=kwargs.get('epsilon', None),
                         state_num=kwargs.get('state_num', None),
                         curr_state=kwargs.get('curr_state', None),
                         replay_type=replay_type,
                         su_event=kwargs.get('su_event', None),
                         replay_thresh=kwargs.get('replay_thresh', None),
                         max_replay=kwargs.get('max_replay', None),
                         epist_decision=kwargs.get('epist_decision', None),
                         epist_replay=kwargs.get('epist_replay', None),
                         rew_weight=kwargs.get('rew_weight', None),
                         trans_weight=kwargs.get('trans_weight', None),
                         eta=kwargs.get('eta', None),
                         replay_update=kwargs.get('replay_update', None))
        self._alpha = alpha

    def __temporal_difference_error__(self, s: int, u: int, s_prime: int, r: float) -> float:
        """
        The TDE algorithm. It simply updates the Q  values in a model-free fashion, and returnns the prediciton
        error
        :param s: current state label
        :param u: chosen action
        :param s_prime: arrival state label
        :param r: received reward
        :return: prediction error
        """
        TD_error = r + self._gamma * max(self._Q[s_prime, :]) - self._Q[s, u]
        self._Q[s, u] += self._alpha * TD_error
        return TD_error

    def agent_type(self) -> None | str:
        """
        Simple getter for the agent type
        :return: 'MB' or 'MF'
        """
        return 'MF'


class MBagent(RLagent):
    """
    Child class of the RLagent, performing model-based updates. Unlike its predecessors, it will compute a transition
    function, an expected reward function, and it will be able to update its Q values via the value/policy iteration
    algorithms
    """

    def __init__(self, act_num: int, max_rew: int, model_type: str, gamma: float, decision_rule: str, kappa: float, **kwargs):
        """
        Consturctor for the model based agent. It will store more data, than its predecessors.
        Exceptions: ValueError, if the str parameters are invalid
        :param act_num: the number of available actions
        :param max_rew: the maximal reward value
        :param model_type: temporal difference, value iteration or policy iteration
        :param gamma: discounting factor
        :param kappa: decay factor of EWMA computation of the reward function
        :param kwargs: parameters regarding the decision rule, the replay and the epistemic rewards
            Regarding the model:
                beta: for softmax decisions (non-optional) [float]
                epsilon: for epsilon-greedy decisions (non-optional) [float]
                state_num: number of states in the environment [int] or None for unknown environment
                    curr_state: if the state_num is None, we need to know the starting state [int]
            Regarding the replay:
                replay_type: "forward", "backward", "priority", "trsam", "bidir" (optional) [str]
                    su_event: in the memory buffer, does a state-action couple constitute an event [True] or only a
                        state [False] (required, unless replay_type is "trsam")
                    replay_thresh: the replay threshold (minimum NORMALIZED error, optional) [float]
                    max_replay: the number of replay steps we'll take (optional) [int]
                    add_predecessors: if replay type is "priority" or "bidir", should I perform a predecessor search
                        when I add an element to the memory buffer ["act": after a real action, "rep": after a replay
                        step, "both": after both] (optional)
                    forbidden_walls: if replay_type is trsam or bidir, is bumping into a wall forbidden during
                        simulation [True, default] or not [False]
            Regarding the epistemic values:
                epist_decision: what epistemic value contributes to decision-making ("rew" for reward uncertainty,
                    "trans" for transition uncertainty, "both" for both, optional)
                epist_replay: IF REPLAY IS PRIORITY, what epistemic value contributes to replay ("rew",
                    "trans" or "both", optional)
                    rew_weight: if "rew" or "both", what is the weight of the reward-related uncertainty-based epistemic
                        reward (required) [float between 0 and 1, weight of Q values is 1-rew_weight-trans_weight]
                    trans_weight: if "trans" or "both", what is the weight of the transition-related uncertainty-based
                        epistemic reward (required) [float between 0 and 1, weight of Q values is
                        1-rew_weight-trans_weight]
                    eta: what is the learning rate for epistemic rewards (required) [float]
                    replay_update: what epistemic values do we update during replay ("rew", "trans", "both", optional)
        """
        if model_type not in ["VI", "PI"]:
            raise ValueError("Model type '" + self._model_type + "' is not supported by the MB agent.")
        RLagent.__init__(self, act_num, max_rew, model_type, gamma, decision_rule,
                         beta=kwargs.get('beta', None),
                         epsilon=kwargs.get('epsilon', None),
                         state_num=kwargs.get('state_num', None),
                         curr_state=kwargs.get('curr_state', None),
                         replay_type=kwargs.get('replay_type', None),
                         su_event=kwargs.get('su_event', None),
                         replay_thresh=kwargs.get('replay_thresh', None),
                         max_replay=kwargs.get('max_replay', None),
                         epist_decision=kwargs.get('epist_decision', None),
                         epist_replay=kwargs.get('epist_replay', None),
                         rew_weight=kwargs.get('rew_weight', None),
                         trans_weight=kwargs.get('trans_weight', None),
                         eta=kwargs.get('eta', None),
                         replay_update=kwargs.get('replay_update', None))
        if self._replay_type in ["priority", "bidir"]:
            self._add_predecessors = kwargs.get("add_predecessors", None)
            if self._add_predecessors not in [None, "act", "rep", "both"]:
                raise ValueError("Predecessors can be added after action 'act', replay 'rep', or both 'both'.")
        if self._replay_type is not None:
            self._forbidden_walls = kwargs.get('forbidden_walls', True)
        self._T = np.ones((self._nS, self._nU, self._nS)) / self._nS  # Transition probs
        self._R = np.zeros((self._nS, self._nU))  # Reward function
        self._N = np.zeros((self._nS, self._nU))  # Number of visits
        self._kappa = kappa  # the decay factor of the exponentially weighted moving average used to represent the
        # rewards

    def __MB_update__(self, s: int, u: int, val_func: np.ndarray) -> np.ndarray:
        """
        The heart of the model based update, computing the new Q value. Returns said value in a one-by-one array
        :param s: current state label
        :param u: action taken
        :param val_func: value function by which we update (Q or V, depending the self._model_type)
        :return: the new Q value in a 1-by-1 array
        """
        return self._R[s, u] + self._gamma * np.dot(np.reshape(self._T[s, u, :], (1, self._nS)), val_func)

    def __policy_iteration__(self, s: int, u: int) -> Tuple[float, np.ndarray]:
        """
        The policy iteration algorithm.
        :param s: current state
        :param u: chosen action
        :return: the difference in the Q value after update
        """
        # TODO algorithm does not converge -- we need to add an update to the policy
        R = np.reshape(np.array([self._R[idx_s, self._pi[idx_s]] for idx_s in range(self._nS)]), (self._nS, 1))
        T = np.array([[self._T[idx_s, self._pi[idx_s], idx_s_prime] for idx_s_prime in range(self._nS)]
                      for idx_s in range(self._nS)])
        V = np.linalg.lstsq(np.eye(self._nS) - np.dot(self._gamma, T), R, rcond=None)
        Q_old = self._Q[s, u]
        self._Q[s, u] = self.__MB_update__(s=s, u=u, val_func=V[0])
        return self._Q[s, u] - Q_old, V[0]

    def __value_iteration__(self, s: int, u: int) -> Tuple[float, np.ndarray]:
        """
        The value iteration algorithm
        :param s: current state
        :param u: action chosen
        :return: the difference in the Q value after update
        """
        Q_max = np.amax(self._Q, axis=1, keepdims=True)
        Q_old = self._Q[s, u]
        self._Q[s, u] = self.__MB_update__(s=s, u=u, val_func=Q_max)
        return self._Q[s, u] - Q_old, np.amax(self._Q, axis=1, keepdims=True)

    def __isrewarded__(self, s: int) -> bool:
        """
        Returns whether a given state is rewarded or not (assuming that every real state transition is more
        probable than 1/nS)
        :param s: The state in question
        :return: Rewarded [True] or not [False]
        """
        for u_pred in range(self._nU):
            s_pred = self._T[:, u_pred, s]
            s_pred = np.nonzero(s_pred > (1 / self._nS))
            if s_pred[0].size != 0 and np.any(self._R[s_pred, u_pred] > 0):  # if we can get here AND rewarded
                return True
        return False

    def __find_predecessors__(self, s: int, val_fun: np.ndarray) -> None:
        """
        It can theoretically speed up the prioritized replay-based learning, to add predecessor states to the memory,
        so that is exactly what this function does. For a given state, we look at all predecessors, that are seemingly
        more likely than chance levels, and add them to the buffer.
        :param s: current state
        :param val_fun: value function by which we update (Q or V, depending the self._model_type)
        :return: -
        """
        # We will have to combine the epistemic and the Q values, should that be required
        combine = self._epist_replay  # it will only really be used to influence the memory_buffer

        # val_fun can either be the value or the Qmax function, depending on what method we choose to use
        for u_pred in range(self._nU):
            # 1) For every possible "predecessor" step, leading to s we find all predecessor states
            s_mask = self._T[:, u_pred, s] > 1 / self._nS
            s_all = np.array([s_idx for s_idx in range(self._T.shape[0])])
            s_all = s_all[s_mask]
            for s_pred in s_all:
                # 2) For all predecessor states we check if a HYPOTHETICAL UPDATE would result in a significant delta

                # We have to check if this one is not the rewarded state, otherwise we might ruin everything
                if self.__isrewarded__(s_pred):
                    continue

                # Compute the HYPOTHETICAL change (not an actual update, just computes the Q-val)
                Q_pred = self.__MB_update__(s_pred, u_pred, val_fun)[0, 0]  # watch out, it returns with a 1x1 array
                delta_Q = Q_pred - self._Q[s_pred, u_pred]

                # Let us combine the delta value (should we need to) with HYPOTHETICAL changes in entropy (no update)
                delta_H_rew, delta_H_trans = None, None
                if combine in ['rew', 'both']:
                    p_rew_pred = self.__epist_update__(s_pred, u_pred, r=self._R[s_pred, u_pred], update=False)
                    H_rew_pred = entropy(np.array([p_rew_pred, 1 - p_rew_pred]))
                    delta_H_rew = H_rew_pred - self.__su_entropy__(s_pred, u_pred, 'rew')
                if combine in ['trans', 'both']:
                    p_trans_pred = self.__epist_update__(s_pred, u_pred, s_prime=s, update=False)
                    H_trans_pred = entropy(p_trans_pred)
                    delta_H_trans = H_trans_pred - self.__su_entropy__(s_pred, u_pred, 'trans')
                comb_delta_Q = self.__combine_Q_epist__(delta_Q, H_rew=delta_H_rew, H_trans=delta_H_trans)

                if abs(comb_delta_Q) > self._replay_thresh:
                    # I only call "store" when the element is above the surprise threshold.
                    # The reason for this is that __store_in_memory__ will delete the previously existing instance of
                    # the current event from the memory buffer if the newest instance of it that we're trying to add is
                    # sub-threshold. Now while this behavior is desirable when we re-visit a state and learn it's not
                    # surprising anymore; we don't want this to be the case, when the new instance comes from a
                    # predecessor-search instead of a real experience
                    self.__store_in_memory__(s_pred, u_pred, s, self._R[s_pred, u_pred], comb_delta_Q)

    def __find_good_actions__(self, s: int, **kwargs) -> np.ndarray:
        """
        Finds the desired (or at least possible) actions from a given state based on the model.
        :param s: The current state
        :param kwargs:
            prev_s: state we don't want to visit if possible
        :return: an array of the possible actions
        """
        prev_s = kwargs.get('prev_s', None)
        # 1) We choose an action based on the model -- this should not be the action to go back, unless necessary
        # What is really important here is that we want to take a step that doesn't just take us back to our last
        # state, unless that is absolutely inevitable. Furthermore, we should be able to notice if we cannot predict
        # where the agent might end up next -- this means that this particular step has never been taken during
        # learning, and thus it might as well be illegal, so it should also be avoided, if possible.
        u_s_prime = self._T[s, :, :]  # The matrix telling us the transition probabilities
        poss_moves = np.array(range(self._nU))  # moves that are physically possible, as far as we know
        # 1.a) let's remove all illegal steps (according to our knowledge)
        for u_idx in range(self._nU):
            illegal = self._N[s, u_idx] == 0  # haven't taken this step yet
            if self._forbidden_walls:
                illegal = illegal or np.argmax(u_s_prime[u_idx, :]) == s
            if illegal:
                poss_moves = np.delete(poss_moves, poss_moves == u_idx)
        # 1.b) now remove the steps that'd take us back to the previous or current state (if we can)
        good_moves = np.copy(poss_moves)  # moves that are possible AND take us further, as far as we know
        stay_in_place = True
        for u_idx in poss_moves:
            if np.argmax(u_s_prime[u_idx, :]) == prev_s:  # If I were to go back, delete
                good_moves = np.delete(good_moves, good_moves == u_idx)
            elif np.argmax(u_s_prime[u_idx, :]) != s:  # If this move allows for actual motion, remember that
                stay_in_place = False
        # 1.c) if nothing is left, let's backtrack
        if len(good_moves) > 0 and not stay_in_place:  # if we have good moves that don't keep us in place
            actions_to_choose = good_moves
        elif len(poss_moves) > 0:  # if we have possible moves (even if bad)
            actions_to_choose = poss_moves
        else:
            actions_to_choose = np.array(range(self._nU))
        return actions_to_choose

    def __trajectory_sampling__(self, s: int, **kwargs) -> None:
        """
        This function performs basic trajectory sampling
        :param s: state we start the simulation from
        :param kwargs:
            stop_loc: state(s) we terminate in [np.ndarray]
            steps: how many steps we are going to simulate [int]
        :return:
        """
        stop_loc = kwargs.get('stop_loc', np.array([]))
        steps = kwargs.get('steps', self._max_replay)
        forbidden_walls = self._trsam_forbidden_walls  # Works MUCH better if true

        # 1) We start from the agent's position
        curr_s = s  # current state
        prev_s = s  # previous state
        max_delta = 0
        it = 0
        while it < steps:
            # 2.a) finding the possible actions
            actions_to_choose = self.__find_good_actions__(curr_s, prev_s=prev_s)

            # 2.b) committing to a choice
            u = self.choose_action(curr_s, actions_to_choose, virtual=True)
            # If we need to combine delta, we choose action based on epist values, otherwise not (done in choose_action)
            s_prime = np.random.choice(list(range(self._nS)), p=self._T[curr_s, u, :])

            # 3) And we get a reward
            r = self._R[curr_s, u]

            # 4) We learn
            delta_Q = self.learnQvalues(curr_s, u, s_prime, r, virtual=True, update_buffer=False)
            # We don't update the buffer, as the path we're taking is completely imaginary

            # 5) And we consider one step to be done
            it += 1

            # 6) Handle delta and if this state is rewarded (and delta is significant), we go back to start
            if abs(delta_Q) > abs(max_delta):
                max_delta = delta_Q
            if curr_s != s_prime:  # If we stay in place, let's not update anything
                prev_s = curr_s
                curr_s = s_prime
            if curr_s in stop_loc or r > 0 or self.__isrewarded__(curr_s):
                if abs(max_delta) > self._replay_thresh:
                    curr_s = s
                    max_delta = 0
                else:
                    break

    def model_tuning(self, s: int, u, s_prime: int, r: int) -> None:
        """
        Tuning of the agent's model parameters
        :param s: current state label
        :param u: chosen action
        :param s_prime: arrival state label
        :param r: the received reward
        :return:
        """
        # First we might need to extend the model if s_prime is never before seen and if the environment is unknown
        try:
            if s_prime not in self._states:
                self.__extend_state_space__(s_prime)
        except AttributeError:
            pass

        # Since s comes from the environment
        s, s_prime = self.__translate_s__(s), self.__translate_s__(s_prime)

        # Then we just update
        self._N[s, u] += 1
        self._T[s, u, :] = (1 - 1 / self._N[s, u]) * self._T[s, u, :] \
                           + np.reshape(np.array(range(self._nS)) == s_prime, (1, 1, self._nS)) / self._N[s, u]
        # self._R[s, u] = (1 - 1 / self._N[s, u]) * self._R[s, u] + r / self._N[s, u]  # stochastic averaging
        # self._R[s, u] = r  # for faster and more efficient testing  # deterministic (last instance)
        self._R[s, u] = self._kappa * r + (1 - self._kappa) * self._R[s, u]  # exponentially weighted moving average
        return

    def agent_type(self) -> None | str:
        """
        Simple getter for the agent type
        :return: 'MB' or 'MF'
        """
        return 'MB'
