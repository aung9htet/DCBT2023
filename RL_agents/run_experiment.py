# Run this script to launch an experiment with predefined parameters
from meta_agent import *

def MF_MB_spatial_navigation() -> None:
    """
    This function realizes the simple navigation by creating a meta agent and passing the messages from the environment
    to it
    Returns:
    """
    ####################################################################################################################
    # Let's start by defining the MF agent's parameters
    # TODO add MF agent parameters here
    MF_params = dict()
    ####################################################################################################################

    ####################################################################################################################
    # Then let's define the MB agent's parameters:
    MB_params = dict()

    # About saving
    MB_params['save_data'] = False  # -------------------- Should save the steps taken into a csv?
    if MB_params['save_data']:
        save_path = './savedata'  # ---------------------- Where should I save
        save_tag = None  # ------------------------------- What tag should I put on saved data

    # About the agent
    MB_params['act_num'] = 3  # -------------------------- Size of action space # TODO make it adaptive
    MB_params['max_rew'] = 1  # -------------------------- The maximal reward in the environment # TODO make it adaptive
    MB_params['model_type'] = 'VI'  # -------------------- We use value iteration as a model type
    MB_params['kappa'] = 0.5  # -------------------------- Learning rate for the model
    MB_params['gamma'] = 0.9  # -------------------------- Discounting factor
    MB_params['decision_rule'] = 'max'  # ---------------- Greedy decisions (could be 'max', 'softmax', 'epsilon')
    if MB_params['decision_rule'] == 'epsilon':
        epsilon = 0.1  # --------------------------------- Epsilon of the epsilon-greedy
    elif MB_params['decision_rule'] == 'softmax':
        beta = 100  # ------------------------------------ Beta for softmax
    MB_params['replay_type'] = 'priority'  # ------------- Replay ('priority', 'trsam', 'bidir', 'backwards', 'forward')
    MB_params['su_event'] = False  # --------------------- What constitutes an event (state-action: True; state: False)
    MB_params['replay_thresh'] = 0.01  # ----------------- Smallest surprise necessary to initiate replay
    MB_params['max_replay'] = 50  # ---------------------- Max replay steps per replay event
    MB_params['add_predecessors'] = 'both'  # ------------ When should I add state predecessors (None, act, rep or both)
    MB_params['forbidden_walls'] = True  # --------------- If we replay (simulate), is bumping into a wall forbidden?
    MB_params['epist_decision'] = 'both'  # -------------- What epistemic value contributes to action selection
    MB_params['epist_replay'] = 'both'  # ---------------- What epist val contributes to replay (rew, trans, both)
    MB_params['rew_weight'] = 0.1  # --------------------- The weight of reward entropy [0-1, float]
    MB_params['trans_weight'] = 0.05  # ------------------ The weight of transition entropy [0-1, float]
    MB_params['eta'] = 0.5  # ---------------------------- Learning rate for the epistemic values
    MB_params['replay_update'] = 'trans'  # -------------- What epistemic val we update during replay (rew, trans, both)
    ####################################################################################################################

    ####################################################################################################################
    # Initializing the agent
    T_Swift = metaAgent(MF_params=MF_params, MB_params=MB_params)
    ####################################################################################################################
