import numpy as np

from MB_agent import *

class metaAgent():
    """
    This agent will be capable of making decisions based on the combined outputs of the MB and the sequential MF agents
    """

    def __init__(self, MF_params: dict, MB_params: dict) -> None:
        # TODO add the model free params
        MB = MBagent(**MB_params)
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
        # TODO implement the learning
        pass

