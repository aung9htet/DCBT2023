# Run this script to launch an experiment with predefined parameters
from meta_agent import *

def MF_MB_spatial_navigation() -> None:
    """
    This function realizes the simple navigation by creating a meta agent and passing the messages from the environment
    to it
    Returns:
    """
    ####################################################################################################################
    # Parameters of the experiment
    # TODO rewise these parameters, potentially adaptively
    steps = 50
    ####################################################################################################################

    ####################################################################################################################
    # Let's start by defining the MF agent's parameters
    # TODO add MF agent parameters here
    MF_params = dict()
    ####################################################################################################################

    MF_params['action_space']= 3

    # About the agent
    MF_params['pl']= 2
    MF_params['stm']= 15                      # default: 50, test: 10
    MF_params['ltm']= 250                     # default: 50K, test: 10K
    MF_params['sequential_bias']= True        # default: True
    MF_params['sequential_value']= 0.02       # default animalai: 0.01
    MF_params['forget_mode']= 'RWD-SING'       # types = ['FIFO-SING', 'FIFO-PROP', 'RWD-SING', 'RWD-PROP', 'LRU-SING', 'LRU-PROP', 'LRU-PROB'] - default: FIFO-SING
    MF_params['forget_ratio']= 0.1            # default = 0.1
    MF_params['retrieval']= 'default'         # types = ['default', 'k_neighbors']
    MF_params['similarity_threshold']= 0.995               # DEFAULT: 0.1, for ERLAM RPs dim 4: 0.0000001, DEFAULT for SEC AE dim 20: 0.2, SEX initial: 0.1
    MF_params['k_neighbors']= 1               # DEFAULT FOR MFEC IN ATARI - 11 NEIGHBORS PER ACTION BUFFER!
    MF_params['value_function']= 'noDist'    # value_functions = ['default', 'noGi', 'noDist', 'noRR', 'soloGi', 'soloDist', 'soloRR']
    MF_params['reward_decay']= 0.9            # default animalai: 0.9
    MF_params['softmax']= False               # originally = False, ATARI = True
    MF_params['selection_mode']= 'argmax'     # selection_mode = ['default', 'argmax']
    MF_params['exploration_mode']= 'epsilon'  # exploration_mode = ['default', 'epsilon', 'epsilon_decay']
    MF_params['exploration_steps']= 150      # THE UNITS ARE NUMBER OF AGENT STEPS! - NatureDQN: 50k STEPS / 50 EPISODES ANIMALAI
    MF_params['epsilon']= 0.1                 # DEFAULT FOR MFEC IN ATARI: 0.1
    MF_params['load_ltm']=  False

    ####################################################################################################################
    # Then let's define the MB agent's parameters:
    MB_params = dict()

    # About saving
    MB_params['save_data'] = False  # -------------------- Should save the steps taken into a csv?
    if MB_params['save_data']:
        MB_params['save_path'] = './savedata'  # --------- Where should I save
        MB_params['save_tag'] = None  # ------------------ What tag should I put on saved data

    # About the agent
    MB_params['act_num'] = 3  # -------------------------- Size of action space # TODO make it adaptive
    MB_params['max_rew'] = 1  # -------------------------- The maximal reward in the environment # TODO make it adaptive
    MB_params['model_type'] = 'VI'  # -------------------- We use value iteration as a model type
    MB_params['kappa'] = 0.5  # -------------------------- Learning rate for the model
    MB_params['gamma'] = 0.9  # -------------------------- Discounting factor
    MB_params['decision_rule'] = 'max'  # ---------------- Greedy decisions (could be 'max', 'softmax', 'epsilon')
    if MB_params['decision_rule'] == 'epsilon':
        MB_params['epsilon'] = 0.1  # -------------------- Epsilon of the epsilon-greedy
    elif MB_params['decision_rule'] == 'softmax':
        MB_params['beta'] = 100  # ----------------------- Beta for softmax
    MB_params['state_num'] = None  # --------------------- The size of the state space
    MB_params['curr_state'] = np.array([0, 0])  # -------- The current location of the agent # TODO make it flexible
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

    ####################################################################################################################
    # Running the experiment
    state = env.reset() # Hypothetical intial state given by the environment

    for _ in range(steps):
        # 1) Ask the environment where we are
        # TODO get the state
        #state = None

        # 2) Choose an action
        # TODO write an action selection algorithm for the meta_agent
        action = T_Swift.action_selection(state)

        # 3) Commit to action
        # TODO take the step using the robot
        #reward = None
        #new_state = None
        new_state, reward, done = env.step(action)

        # 4) Learn
        # TODO implement the learning via the meta agent (replay included)
        T_Swift.learning()

        # 5) If the agent reached a reward, send it back to the starting position
        # TODO implement the agent backtracking. Potentially replay while doing so
        if done:
            state = env.reset()
            T_Swift.reset()
        else:
            state = new_state
    ####################################################################################################################
