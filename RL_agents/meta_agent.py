from MB_agent import *

class metaAgent():
    """
    This agent will be capable of making decisions based on the combined outputs of the MB and the sequential MF agents
    """

    def __init__(self, MF_params: dict, MB_params: dict) -> None:
        # TODO add the model free params
        MB = MBagent(**MB_params)
        return

