import numpy as np

from MB_agent import *
from SEC_agent import *

class metaAgent():
    """
    This agent will be capable of making decisions based on the combined outputs of the MB and the sequential MF agents
    """

    def __init__(self, MF_params: dict, MB_params: dict) -> None:
        self._replay_thresh = MB_params['replay_thresh']
        self._actions = {MB_params['actions'][idx]: int(idx) for idx in range(len(MB_params['actions']))}
        self._states = np.array([[MB_params['curr_state']]])
        MB_params['curr_state'] = self.__decode_state__(MB_params['curr_state'])
        self._MF = SECagent(**MF_params)
        self._MB = MBagent(**MB_params)
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
            self._states = np.append(self._states, np.array([[state]]), axis=0)
        return

    def __decode_state__(self, state: np.ndarray) -> int:
        """
        Decodes the sate into integers for the MB agent
        Args:
            state:

        Returns:

        """
        # return np.where((self._states == np.array([state])).all(axis=1))[0][0]
        # TODO this is only for the virtual version
        return state[0]

    # Public methods
    def action_selection(self, state: np.ndarray, poss_moves: np.ndarray) -> Tuple[str, bool]:
        """
        Performs the action selection comparing the output of the 2 agents. The one that shows a higher Q value wins.
        Args:
            state: The state I am currently in (coordinates)
            poss_moves: What are the currently available moves (strings)

        Returns:
            'straight', 'left', or 'right' as the chosen action; and
            True if we used the MF agent, False if we used the MB agent
        """
        # 1) MF action selection
        #action_MF, Q_MF = self.MF.action_selection(state)
        action_MF, Q_MF = self._MF.choose_action(state)

        # 2) MB action selection
        decoded_state = self.__decode_state__(state)
        action_MB, Q_MB = self._MB.choose_action(decoded_state,
                                                 np.array([self._actions[move] for move in poss_moves]))

        # 3) Compare results
        if Q_MF >= Q_MB:
            return list(self._actions.keys())[list(self._actions.values()).index(action_MF)], True
            # return list(self._actions.keys())[list(self._actions.values()).index(action_MB)], False
        else:
            return list(self._actions.keys())[list(self._actions.values()).index(action_MB)], False

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
        # Update SEC's STM based on previous (state,action) couplet
        self._MF.update_STM(sa_couplet=[state, self._actions[action]])
        self._MF.update_sequential_bias()
        self._MF.update_LTM(reward)

        # 2) Teach the MB agents
        decoded_state = self.__decode_state__(state)
        decoded_new_state = self.__decode_state__(new_state)
        self._MB.model_tuning(decoded_state, self._actions[action], decoded_new_state, reward)
        delta_C = self._MB.learnQvalues(decoded_state, self._actions[action], decoded_new_state, reward)
        replayed = False
        if self._replay_thresh is not None and delta_C > self._replay_thresh:
            self._MB.memory_replay()
            replayed = True

        # 3) Return
        return replayed


    def reset(self) -> None:
        """
        Reset the short-term memory of the MF agent
        """
        self._MF.reset_memory()

    def toggle_save(self):
        """
        Toggles saving for future visualization
        Args:

        Returns:

        """
        self._MB.toggle_save()
        return

    def dump_agent(self, **kwargs):
        """
        Saving the MB agent
        Args:
            **kwargs:
                path: Path to save. If undefined we save to the working folder
                tag: the tag to add to the file [optional str]
        Returns:

        """
        self._MB.dump_agent(path=kwargs.get('path', None), label=kwargs.get('label', None))
        return

