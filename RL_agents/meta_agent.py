import numpy as np

from MB_agent import *

class metaAgent():
    """
    This agent will be capable of making decisions based on the combined outputs of the MB and the sequential MF agents
    """

    def __init__(self, MF_params: dict, MB_params: dict) -> None:
        # TODO add the model free params
        self._MB = MBagent(**MB_params)
        self._replay_thresh = MB_params['replay_thresh']
        self._actions = {int(0): 'forward', int(1): 'left', int(2): 'right'}
        self._states = {MB_params['curr_state']: int(0)}
        return

    # Private methods
    def __add_new_state__(self, state: np.ndarray) -> None:
        """
        Upon encountering a new state, add it to the _states dictionary, so that the agent can translate it into ints
        Args:
            state: the coordinates of the newly encountered state

        Returns:

        """
        if state not in self._states:
            self._states[state] = len(self._states)
        return

    # Public methods
    def action_selection(self, state: np.ndarray) -> Tuple[str, bool]:
        """
        Performs the action selection comparing the output of the 2 agents. The one that shows a higher Q value wins.
        Args:
            state: The state I am currently in (coordinates)

        Returns:
            'straight', 'left', or 'right' as the chosen action; and
            True if we used the MF agent, False if we used the MB agent
        """
        # TODO implement the decision
        pass

    def learning(self, state: np.ndarray, action: str, new_state: np.ndarray, reward: float) -> bool:
        """
        The learning performed by both sub-agents
        Args:
            state: the state that the agent started from
            action: the action the agent took
            new_state: the state the agent arrived in
            reward: the reward the agent gained

        Returns:
            True if the agent performed replay, False if not
        """
        # 0) First if we just encountered a brand-new state, let's add it to our dictionary
        if new_state not in self._states:
            self.__add_new_state__(new_state)

        # 1) Teach the MF agent
        # TODO MF learning

        # 2) Teach the MB agents
        delta_C = self._MB.learnQvalues(self._states[state],
                                        list(self._actions.keys())[list(self._actions.values()).index(action)],
                                        self._states[new_state],
                                        reward)
        replayed = False
        if self._replay_thresh is not None and delta_C > self._replay_thresh:
            self._MB.memory_replay()
            replayed = True

        # 3) Return
        return replayed
