# created by MaRo 2023-06-20
# model of the basal ganglia, further development of the work from Baladron and Hamker
# title: "Habit learning in hierarchical cortex-basal ganglia loops"
# related paper: https://pubmed.ncbi.nlm.nih.gov/32237250/
# and the master thesis of David Simon
# title: "Goal selection in a hierarchical model of the multiple basal ganglia loops"

from ANNarchy import *
from states import *
import random

######################################################################################################
### Neurons of the network ###########################################################################
######################################################################################################

# basic explanations of the neuronal parameters
# tau = time constant
# baseline --> baseline_firing_rate
# lesion --> factor to amplify the membrane potential(mp)
# r --> instantaneous firing rate
# pos() --> defines mp as positive

# model of an input neuron as source for the Hippocampus
# the rate is set to rely on external input -> means: we can set the input manually
input_neuron = Neuron(
    parameters="""
        tau = 1.5
        baseline = 0.0
        noise = 0.0
    """,
    equations="""
        tau * dmp/dt + mp = baseline + noise * Uniform(-1.0, 1.0)
        r = mp : min = 0.0 
    """
)

# linear model of neuron --> example: y = sum(w * x)
linear_neuron = Neuron(
    parameters="""
        tau = 10.0
        baseline = 0.0 
        noise = 0.0
        lesion = 1.0
    """,
    equations="""
        tau * dmp/dt + mp = sum(exc) - sum(inh) + baseline + noise * Uniform(-1.0, 1.0)
        r = lesion * pos(mp)
    """
)

# linear model of neuron with trace
linear_neuron_trace = Neuron(
    parameters="""
        tau = 10.0
        baseline = 0.0
        noise = 0.0
        tau_trace = 120.0
        lesion = 1.0
    """,
    equations="""
        tau * dmp/dt + mp = sum(exc) - sum(inh) + baseline + noise * Uniform(-1.0, 1.0)
        r = lesion * pos(mp)
        tau_trace * dtrace/dt + trace = r
    """
)
# linear model of neuron with ability of learning
linear_neuron_learning = Neuron(
    parameters="""
        tau = 10.0
        baseline = 0.0
        noise = 0.0
        lesion = 1.0
        learning = 0.0
    """,
    equations="""
        cortex_input = if learning == 1.0: sum(exc) else: 0.0
        tau * dmp/dt + mp = cortex_input - sum(inh) + baseline + noise * Uniform(-1.0, 1.0)
        r = lesion * pos(mp) 
    """
)

# dopamine model of neuron
dopamine_neuron = Neuron(
    parameters="""
        tau = 10.0
        firing = 0.0
        inhibition = 0.0
        baseline = 0.0
        exc_threshold = 0.0
        factor_inh = 10.0
    """,
    equations="""
        ex_in = if (sum(exc) > exc_threshold): 1 else: 0
        s_inh = sum(inh)
        aux = if (firing > 0): (ex_in) * (pos(1.0 - baseline - s_inh) + baseline) + (1 - ex_in) * (-factor_inh * sum(inh) + baseline) else: baseline
        tau * dmp/dt + mp = aux
        r = mp : min = 0.0
    """
)

######################################################################################################
### Synapses of the Network ##########################################################################
######################################################################################################

# Dopamin types are distinguished between DA_typ = 1 --> D1 type and DA_typ = -1 --> D2 type
# K_burst controls the strength for LTP, when phasic dopamine falls under the baseline
# K_dip controls the strength for LTD when a reward is obtained

# Excitatory synapse between hippocampus_activity and reward_prediction_error and other synapses of the ventral loop
DAPostCovarianceNoThreshold = Synapse(
    parameters="""
        tau = 1000.0
        tau_alpha = 10.0
        regularization_threshold = 1.0
        baseline_dopa = 0.1
        K_burst = 1.0
        K_dip = 0.4
        DA_type = 1
        threshold_pre = 0.0
        threshold_post = 0.0
    """,
    equations="""
        tau_alpha * dalpha/dt + alpha = pos(post.mp - regularization_threshold)
        dopa_sum = 2.0 * (post.sum(dopa) - baseline_dopa)
        trace = pos(post.r - mean(post.r) - threshold_post) * (pre.r - mean(pre.r) - threshold_pre)
        condition_0 = if (trace > 0.0) and (w > 0.0): 1 else: 0 
        dopa_mod = if (DA_type * dopa_sum > 0): DA_type * K_burst * dopa_sum else: condition_0 * DA_type * K_dip * dopa_sum
        delta = (dopa_mod * trace - alpha * pos(post.r - mean(post.r) - threshold_post) * pos(post.r - mean(post.r) - threshold_post))
        tau * dw/dt = delta : min = 0.0
    """
)
# Excitatory synapse between medial PFC and the gpi_dorsomedial
# change in contrast to the DAPostCovarianceNoThreshold is the calculation of the trace through:
# trace = pos(post.trace -  mean(post.trace) - threshold_post) * (pre.r - mean(pre.r) - threshold_pre)
DAPostCovarianceNoThreshold_trace = Synapse(
    parameters="""
        tau = 1000.0
        tau_alpha = 10.0
        regularization_threshold = 1.0
        baseline_dopa = 0.1
        K_burst = 1.0
        K_dip = 0.4
        DA_type = 1
        threshold_pre = 0.0
        threshold_post = 0.0
    """,
    equations="""
        tau_alpha * dalpha/dt + alpha = pos(post.mp - regularization_threshold)
        dopa_sum = 2.0 * (post.sum(dopa) - baseline_dopa)
        trace = pos(post.trace - mean(post.trace) - threshold_post) * (pre.r - mean(pre.r) -threshold_pre)
        condition_0 = if (trace > 0.0) and (w > 0.0): 1 else: 0
        dopa_mod = if ((DA_type * dopa_sum) > 0): DA_type * K_burst * dopa_sum else: condition_0 * DA_type * K_dip * dopa_sum
        delta = (dopa_mod * trace - alpha * pos(post.r - mean(post.r) -threshold_post) * pos(post.r - mean(post.r) - threshold_post))
        tau * dw/dt = delta : min = 0.0
    """
)

# Excitatory synapse between the stn_ventral and the vp_direct
DAPreCovariance_excitatory = Synapse(
    parameters="""
    tau = 1000.0
    tau_alpha = 10.0
    regularization_threshold = 1.0
    baseline_dopa = 0.1
    K_burst = 1.0
    K_dip = 0.4
    DA_type = 1
    threshold_pre = 0.0
    threshold_post = 0.0
    """,
    equations="""
        tau_alpha * dalpha/dt = pos(post.mp - regularization_threshold) - alpha
        dopa_sum = 2.0 * (post.sum(dopa) - baseline_dopa)
        trace = pos(pre.r - mean(pre.r) - threshold_pre) * (post.trace - mean(post.trace) - threshold_post)
        aux = if (trace < 0.0): 1 else: 0
        dopa_mod = if (dopa_sum > 0): K_burst * dopa_sum else: K_dip * dopa_sum * aux
        delta = dopa_mod * trace - alpha * pos(trace)
        tau * dw/dt = delta : min = 0.0
    """
)

# Inhibitory synapse between the nacc_shell_d1 and the vp_direct
DAPreCovariance_inhibitory = Synapse(
    parameters="""
        tau = 1000.0
        tau_alpha = 10.0
        regularization_threshold = 1.0
        baseline_dopa = 0.1
        K_burst = 1.0
        K_dip = 0.4
        DA_type = 1
        threshold_pre = 0.0
        threshold_post = 0.0
    """,
    equations="""
        tau_alpha * dalpha/dt = pos(-post.mp - regularization_threshold) - alpha
        dopa_sum = 2.0 * (post.sum(dopa) - baseline_dopa)
        trace = pos(pre.r - mean(pre.r) - threshold_pre) * (mean(post.r) - post.r - threshold_post)
        aux = if (trace > 0): 1 else: 0
        dopa_mod = if ((DA_type * dopa_sum) >0): DA_type * K_burst * dopa_sum else: aux * DA_type * K_dip * dopa_sum
        delta = dopa_mod * trace - alpha * pos(trace)
        tau * dw/dt = delta : min = 0.0
    """
)
# Inhibitory synapse between the striatum_d1_dorsomedial and the gpi_dorsomedial
DAPreCovariance_inhibitory_trace = Synapse(
    parameters="""
        tau = 1000.0
        tau_alpha = 10.0
        regularization_threshold = 1.0
        baseline_dopa = 0.1
        K_burst = 1.0
        K_dip = 0.4
        DA_type = 1
        threshold_pre = 0.0
        threshold_post = 0.0
        negterm = 1
    """,
    equations="""
        tau_alpha * dalpha/dt = pos(-post.mp - regularization_threshold) - alpha
        dopa_sum = 2.0 * (post.sum(dopa) - baseline_dopa)
        trace = pos(pre.trace - mean(pre.trace) - threshold_pre) * (mean(post.r) - post.r - threshold_post)
        aux = if (trace > 0): negterm else: 0
        dopa_mod = if (DA_type * dopa_sum > 0): K_burst * dopa_sum else: aux * DA_type * K_dip * dopa_sum
        delta = dopa_mod * trace - alpha * pos(trace)
        tau * dw/dt = delta : min = 0 
    """
)
# reversed synapse mostly used for local inhibitory connection
ReversedSynapse = Synapse(
    parameters="""
        reversal = 0.3
    """,
    psp="""
        w * pos(reversal - pre.r)
    """
)
# synapse connection between the reward_prediction_error and the vta of the ventral loop
DAPrediction = Synapse(
    parameters="""
        tau = 100000.0
        baseline_dopa = 0.1
    """,
    equations="""
       aux = if (post.sum(exc) > 0.001): 1.0 else: 2.5
       delta = aux * (post.r - baseline_dopa) * pos(pre.r - mean(pre.r))
       tau * dw/dt = delta : min = 0.0
    """
)

######################################################################################################
### Populations of the ventral loop ##################################################################
######################################################################################################

# modeled as simple input source
hippocampus = Population(name="hippocampus", geometry=(4, 100), neuron=input_neuron)

# reward independent of the stimulus in the hippocampus
hippocampus_activity = Population(name="hippocampus_activity", geometry=24, neuron=linear_neuron)

# reward_prediction_error of the ventral loop
reward_prediction_error = Population(name="reward_prediction_error", geometry=(4, 6), neuron=linear_neuron)
reward_prediction_error.tau = 10.0
reward_prediction_error.noise = 0.0
reward_prediction_error.baseline = 0.0
reward_prediction_error.lesion = 1.0

# direct pathway d1 of the nucleus accumbens
nacc_shell_d1 = Population(name="nacc_shell_d1", geometry=(4, 6), neuron=linear_neuron_trace)
nacc_shell_d1.tau = 10.0
nacc_shell_d1.noise = 0.3
nacc_shell_d1.baseline = 0.0
nacc_shell_d1.lesion = 1.0
nacc_shell_d1.tau_trace = 100

# indirect pathway d2 of the nucleus accumbens, here designed similar to the direct pathway nacc_shell_d2
nacc_shell_d2 = Population(name="nacc_shell_d2", geometry=(4, 6), neuron=linear_neuron_trace)
nacc_shell_d2.tau = 10.0
nacc_shell_d2.noise = 0.01
nacc_shell_d2.baseline = 0.0
nacc_shell_d2.lesion = 1.0
nacc_shell_d2.tau_trace = 100

# subthalamic nucleus (STN) of the ventral loop
stn_ventral = Population(name="stn_ventral", geometry=(4, 6), neuron=linear_neuron_trace)
stn_ventral.tau = 10.0
stn_ventral.noise = 0.01
stn_ventral.baseline = 0.0
stn_ventral.tau_trace = 100

# ventral pallidum direct, designed similar to the globus pallidus internus (GPi)
vp_direct = Population(name="vp_direct", geometry=6, neuron=linear_neuron_trace)
vp_direct.tau = 10.0
vp_direct.noise = 0.3
vp_direct.baseline = 1.5
vp_direct.tau_trace = 100

# ventral pallidum indirect, designed similar to the globus pallidus externus (GPe)
vp_indirect = Population(name="vp_indirect", geometry=6, neuron=linear_neuron_trace)
vp_indirect.tau = 10.0
vp_indirect.noise = 0.05
vp_indirect.baseline = 1.0
vp_indirect.tau_trace = 100

# ventral tegmental area (VTA) of the ventral loop, important for distributing the reward
ventral_tegmental_area = Population(name="ventral_tegmental_area", geometry=1, neuron=dopamine_neuron)
ventral_tegmental_area.baseline = 0.15
ventral_tegmental_area.factor_inh = 5.0

# pedunculopontine tegmental nucleus (PPTN) of the ventral loop
pptn_ventral = Population(name="pptn_ventral", geometry=1, neuron=input_neuron)
pptn_ventral.tau = 1.0

# cortical feedback from medial PFC to VP (direct and indirect), enables hebbian learning rule by excitation of the VP neurons
cortical_feedback_ventral = Population(name="cortical_feedback_ventral", geometry=6, neuron=linear_neuron_learning)
cortical_feedback_ventral.tau = 10.0
cortical_feedback_ventral.noise = 0.01
cortical_feedback_ventral.baseline = 0.4
cortical_feedback_ventral.lesion = 1.0
cortical_feedback_ventral.learning = 0.0

######################################################################################################
### Populations of the working memory ################################################################
######################################################################################################

# ventral thalamus, connection between ventral loop and working memory
thalamus_ventral = Population(name="thalamus_ventral", geometry=6, neuron=linear_neuron)
thalamus_ventral.tau = 10.0
thalamus_ventral.noise = 0.025 / 2.0
thalamus_ventral.baseline = 1.2

# medial PreFrontalCortex, connection between working memory, ventral loop and dorsomedial loop
medial_pfc = Population(name="medial_pfc", geometry=6, neuron=linear_neuron)
medial_pfc.tau = 40.0
medial_pfc.noise = 0.01

# the core of the nucleus accumbens as part of the working memory
nacc_core = Population(name="nacc_core", geometry=6, neuron=linear_neuron)
nacc_core.tau = 10.0
nacc_core.noise = 0.0
nacc_core.baseline = 0.0
nacc_core.lesion = 1.0

# substantia nigra pars reticulata (SNr) as part of the working memory
substantia_nigra_pars_reticulata = Population(name="substantia_nigra_pars_reticulata", geometry=6, neuron=linear_neuron)
substantia_nigra_pars_reticulata.tau = 10.0
substantia_nigra_pars_reticulata.noise = 0.3
substantia_nigra_pars_reticulata.baseline = 1.0

######################################################################################################
### Populations of the dorsomedial loop (caudate) ####################################################
######################################################################################################

# visual input as simple input source
visual_input = Population(name="visual_input", geometry=(4, 3), neuron=linear_neuron)
visual_input.baseline = 0.0
visual_input.noise = 0.001

# direct pathway d1 of the dorsomedial loop
striatum_d1_dorsomedial = Population(name="striatum_d1_dorsomedial", geometry=(6, 8), neuron=linear_neuron_trace)
striatum_d1_dorsomedial.tau = 10.0
striatum_d1_dorsomedial.noise = 0.4
striatum_d1_dorsomedial.baseline = 0.0
striatum_d1_dorsomedial.lesion = 1.0
striatum_d1_dorsomedial.tau_trace = 100

# indirect pathway d2 of the dorsomedial loop
striatum_d2_dorsomedial = Population(name="striatum_d2_dorsomedial", geometry=(3, 8), neuron=linear_neuron_trace)
striatum_d2_dorsomedial.tau = 10.0
striatum_d2_dorsomedial.noise = 0.01
striatum_d2_dorsomedial.baseline = 0.0
striatum_d2_dorsomedial.lesion = 1.0

# subthalamic nucleus (STN) of the dorsomedialen loop
stn_dorsomedial = Population(name="stn_dorsomedial", geometry=3, neuron=linear_neuron)
stn_dorsomedial.tau = 10.0
stn_dorsomedial.noise = 0.01
stn_dorsomedial.baseline = 0.0

# globus pallidus internus of the dorsomedial loop
gpi_dorsomedial = Population(name="gpi_dorsomedial", geometry=6, neuron=linear_neuron)
gpi_dorsomedial.tau = 10.0
gpi_dorsomedial.noise = 0.5
gpi_dorsomedial.baseline = 1.9

# globus pallidus externus of the dorsomedial loop
gpe_dorsomedial = Population(name="gpe_dorsomedial", geometry=6, neuron=linear_neuron)
gpe_dorsomedial.tau = 10.0
gpe_dorsomedial.noise = 0.05
gpe_dorsomedial.baseline = 1.0

# substantia nigra pars compacta (SNc) of the dorsomedial loop, important for distributing the reward
substantia_nigra_pars_compacta = Population(name="substantia_nigra_pars_compacta", geometry=6, neuron=dopamine_neuron)
substantia_nigra_pars_compacta.exc_threshold = 0.2
substantia_nigra_pars_compacta.baseline = 0.1
substantia_nigra_pars_compacta.factor_inh = 1.0

# pedunculopontine tegmental nucleus (PPTN) of the dorsomedial loop
pptn_dorsomedial = Population(name="pptn_dorsomedial", geometry=6, neuron=input_neuron)

# cortical feedback from dorsolateral PFC to GPi and GPe, enables hebbian learning rule by excitation of the GP neurons
cortical_feedback_dorsomedial = Population(name="cortical_feedback_dorsomedial", geometry=6, neuron=linear_neuron_learning)
cortical_feedback_dorsomedial.tau = 10.0
cortical_feedback_dorsomedial.noise = 0.01
cortical_feedback_dorsomedial.baseline = 0.4
cortical_feedback_dorsomedial.lesion = 1.0
cortical_feedback_dorsomedial.learning = 1.0

# dorsomedial thalamus, connection between dorsomedial loop and dorsal prefrontal cortex
thalamus_dorsomedial = Population(name="thalamus_dorsomedial", geometry=6, neuron=linear_neuron)
thalamus_dorsomedial.tau = 10.0
thalamus_dorsomedial.noise = 0.025
thalamus_dorsomedial.baseline = 0.7

######################################################################################################
### Populations of the dorsolateral loop (putamen) ###################################################
######################################################################################################

# dorsolateral PreFrontalCortex (dPFC), connection between dorsomedial loop and dorsolateral loop
dorsolateral_pfc = Population(name="dorsolateral_pfc", geometry=6, neuron=linear_neuron)
dorsolateral_pfc.tau = 40.0
dorsolateral_pfc.noise = 0.01

# direct pathway d1 of th dorsolateral loop
striatum_d1_dorsolateral = Population(name="striatum_d1_dorsolateral", geometry=6, neuron=linear_neuron)

# globus pallidus internus of the dorsolateral loop
gpi_dorsolateral = Population(name="gpi_dorsolateral", geometry=3, neuron=linear_neuron)
gpi_dorsolateral.baseline = 1.25

# dorsolateral thalamus
thalamus_dorsolateral = Population(name="thalamus_dorsolateral", geometry=3, neuron=linear_neuron)
thalamus_dorsolateral.baseline = 0.85

# premotor as executing part of the multi loop systems --> output
premotor = Population(name="premotor", geometry=3, neuron=linear_neuron)

######################################################################################################
### Projections of the ventral loop ##################################################################
######################################################################################################

## direct pathway ventral ############################################################################
# projection between hippocampus and the nacc_shell_d1 of the ventral loop as direct pathway
hippocampus_to_nacc_shell_d1 = Projection(pre=hippocampus, post=nacc_shell_d1, target="exc", synapse=DAPostCovarianceNoThreshold)
hippocampus_to_nacc_shell_d1.connect_all_to_all(weights=Normal((0.1 / 3), 0.02))  # previous weights = Normal(0.1/(num_stim/2), 0.02)  --> num_stim = 6
hippocampus_to_nacc_shell_d1.tau = 200
hippocampus_to_nacc_shell_d1.regularization_threshold = 1.0
hippocampus_to_nacc_shell_d1.tau_alpha = 5.0
hippocampus_to_nacc_shell_d1.baseline_dopa = 0.15
hippocampus_to_nacc_shell_d1.K_dip = 0.05
hippocampus_to_nacc_shell_d1.K_burst = 1.0
hippocampus_to_nacc_shell_d1.DA_type = 1
hippocampus_to_nacc_shell_d1.threshold_pre = 0.2
hippocampus_to_nacc_shell_d1.threshold_post = 0.0

# local inhibitory connection of the nacc_shell_d1
nacc_shell_d1_to_nacc_shell_d1 = Projection(pre=nacc_shell_d1, post=nacc_shell_d1, target="inh")
nacc_shell_d1_to_nacc_shell_d1.connect_all_to_all(weights=0.16 * 0.166)

# projection between the nacc_shell_d1 and the vp_direct of the ventral loop as the second part of the direct pathway
nacc_shell_d1_to_vp_direct = Projection(pre=nacc_shell_d1, post=vp_direct, target='inh', synapse=DAPreCovariance_inhibitory)
nacc_shell_d1_to_vp_direct.connect_all_to_all(weights=Normal(0.2, 0.01))
nacc_shell_d1_to_vp_direct.tau = 2000
nacc_shell_d1_to_vp_direct.regularization_threshold = 3.5
nacc_shell_d1_to_vp_direct.tau_alpha = 2.5
nacc_shell_d1_to_vp_direct.baseline_dopa = 0.15
nacc_shell_d1_to_vp_direct.K_dip = 0.8
nacc_shell_d1_to_vp_direct.K_burst = 1.2
nacc_shell_d1_to_vp_direct.threshold_post = 0.4
nacc_shell_d1_to_vp_direct.threshold_pre = 0.1
nacc_shell_d1_to_vp_direct.DA_type = 1
nacc_shell_d1_to_vp_direct.negterm = 5.0

# local excitatory connection of the vp_direct
vp_direct_to_vp_direct = Projection(pre=vp_direct, post=vp_direct, target="exc", synapse=ReversedSynapse)
vp_direct_to_vp_direct.connect_all_to_all(weights=0.7)
vp_direct_to_vp_direct.reversal = 0.3

# local excitatory connection of the vp_direct
vp_direct_to_vp_direct = Projection(pre=vp_direct[0:2], post=vp_direct[0:2], target="exc", synapse=ReversedSynapse)
vp_direct_to_vp_direct.connect_all_to_all(weights=0.7)
vp_direct_to_vp_direct.reversal = 0.3

# local excitatory connection of the vp_direct
vp_direct_to_vp_direct = Projection(pre=vp_direct[2:4], post=vp_direct[2:4], target="exc", synapse=ReversedSynapse)
vp_direct_to_vp_direct.connect_all_to_all(weights=0.7)
vp_direct_to_vp_direct.reversal = 0.3

# local excitatory connection of the vp_direct
vp_direct_to_vp_direct = Projection(pre=vp_direct[4:6], post=vp_direct[4:6], target="exc", synapse=ReversedSynapse)
vp_direct_to_vp_direct.connect_all_to_all(weights=0.7)
vp_direct_to_vp_direct.reversal = 0.3

# projection between the vp_direct and the thalamus_ventral of the ventral loop
vp_direct_to_thalamus_ventral = Projection(pre=vp_direct, post=thalamus_ventral, target="inh")
vp_direct_to_thalamus_ventral.connect_one_to_one(weights=0.75)

# local inhibitory connection of the thalamus
thalamus_ventral_to_thalamus_ventral = Projection(pre=thalamus_ventral, post=thalamus_ventral, target="inh")
thalamus_ventral_to_thalamus_ventral.connect_all_to_all(weights=0.2)

## indirect pathway ventral ##########################################################################
# projection between hippocampus and the nacc_shell_d2 of the ventral loop as indirect pathway
hippocampus_to_nacc_shell_d2 = Projection(pre=hippocampus, post=nacc_shell_d2, target="exc", synapse=DAPostCovarianceNoThreshold)
hippocampus_to_nacc_shell_d2.connect_all_to_all(weights=Normal((0.01 / 3), 0.01))  # previous weights = Normal(0.1/(num_stim/2), 0.01)  --> num_stim = 6
hippocampus_to_nacc_shell_d2.tau = 10.0
hippocampus_to_nacc_shell_d2.regularization_threshold = 1.5
hippocampus_to_nacc_shell_d2.tau_alpha = 15.0
hippocampus_to_nacc_shell_d2.baseline_dopa = 0.15
hippocampus_to_nacc_shell_d2.K_dip = 0.2
hippocampus_to_nacc_shell_d2.K_burst = 1.0
hippocampus_to_nacc_shell_d2.DA_type = -1
hippocampus_to_nacc_shell_d2.threshold_pre = 0.2
hippocampus_to_nacc_shell_d2.threshold_post = 0.005

# local inhibitory connection of the nacc_shell_d2
nacc_shell_d2_to_nacc_shell_d2 = Projection(pre=nacc_shell_d2, post=nacc_shell_d2, target="inh")
nacc_shell_d2_to_nacc_shell_d2.connect_all_to_all(weights=0.1)

# projection between the nacc_shell_d2 and the vp_indirect of the ventral loop as the second part of the indirect pathway
nacc_shell_d2_to_vp_indirect = Projection(pre=nacc_shell_d2, post=vp_indirect, target='inh', synapse=DAPreCovariance_inhibitory)
nacc_shell_d2_to_vp_indirect.connect_all_to_all(weights=0.01)
nacc_shell_d2_to_vp_indirect.tau = 1000
nacc_shell_d2_to_vp_indirect.regularization_threshold = 1.5
nacc_shell_d2_to_vp_indirect.tau_alpha = 20.0
nacc_shell_d2_to_vp_indirect.baseline_dopa = 0.15
nacc_shell_d2_to_vp_indirect.K_dip = 0.1
nacc_shell_d2_to_vp_indirect.K_burst = 1.0
nacc_shell_d2_to_vp_indirect.threshold_post = 0.0
nacc_shell_d2_to_vp_indirect.threshold_pre = 0.25
nacc_shell_d2_to_vp_indirect.DA_type = -1

# projection between the vp_direct and the vp_indirect as the indirect pathway
vp_indirect_to_vp_direct = Projection(pre=vp_indirect, post=vp_direct, target="inh")
vp_indirect_to_vp_direct.connect_one_to_one(weights=0.5)

## hyperdirect pathway ventral #######################################################################
# projection between medial PFC and the subthalamic nucleus (STN) of the ventral loop as hyperdirect pathway
medial_pfc_to_stn_ventral = Projection(pre=medial_pfc, post=stn_ventral, target='exc', synapse=DAPostCovarianceNoThreshold)
medial_pfc_to_stn_ventral.connect_all_to_all(weights=Uniform(0.0, 0.001))
medial_pfc_to_stn_ventral.tau = 1500.0
medial_pfc_to_stn_ventral.regularization_threshold = 1.0
medial_pfc_to_stn_ventral.tau_alpha = 15.0
medial_pfc_to_stn_ventral.baseline_dopa = 0.15
medial_pfc_to_stn_ventral.K_dip = 0.0
medial_pfc_to_stn_ventral.K_burst = 1.0
medial_pfc_to_stn_ventral.DA_type = 1
medial_pfc_to_stn_ventral.threshold_pre = 0.15

# local inhibitory connection of the stn_ventral
stn_ventral_to_stn_ventral = Projection(pre=stn_ventral, post=stn_ventral, target="inh")
stn_ventral_to_stn_ventral.connect_all_to_all(weights=0.3)

# projection between the stn_ventral and the vp_direct of the ventral loop as the second part of the hyperdirect pathway
stn_ventral_to_vp_direct = Projection(pre=stn_ventral, post=vp_direct, target="exc", synapse=DAPreCovariance_excitatory)
stn_ventral_to_vp_direct.connect_all_to_all(weights=Uniform(0.01, 0.001))
stn_ventral_to_vp_direct.tau = 9000
stn_ventral_to_vp_direct.regularization_threshold = 3.5
stn_ventral_to_vp_direct.tau_alpha = 1.0
stn_ventral_to_vp_direct.baseline_dopa = 0.15
stn_ventral_to_vp_direct.K_dip = 0.4
stn_ventral_to_vp_direct.K_burst = 1.0
stn_ventral_to_vp_direct.threshold_post = -0.15
stn_ventral_to_vp_direct.DA_type = 1

## learning part #####################################################################################
# plastic connections for the cells encoding the reward prediction
# projection between hippocampus and hippocampus_activity
hippocampus_to_activity = Projection(pre=hippocampus, post=hippocampus_activity[:8], target="exc")
hippocampus_to_activity.connect_all_to_all(weights=0.125)

# projection between hippocampus_activity and reward_prediction_error
activity_to_rpe = Projection(pre=hippocampus_activity, post=reward_prediction_error, target="exc", synapse=DAPostCovarianceNoThreshold)
activity_to_rpe.connect_all_to_all(weights=Normal((0.1 / 3), 0.02))  # previous weights = Normal(0.1/(num_stim/2), 0.02)  --> num_stim = 6
activity_to_rpe.tau = 200
activity_to_rpe.regularization_threshold = 1.0
activity_to_rpe.tau_alpha = 5.0
activity_to_rpe.baseline_dopa = 0.15
activity_to_rpe.K_dip = 0.05
activity_to_rpe.K_burst = 1.0
activity_to_rpe.DA_type = 1
activity_to_rpe.threshold_pre = 0.2
activity_to_rpe.threshold_post = 0.0

# local projection within the reward_prediction_error
rpe_to_rpe = Projection(pre=reward_prediction_error, post=reward_prediction_error, target="inh")
rpe_to_rpe.connect_all_to_all(weights=(0.16 * 0.166))  # previous weights = weight_local_inh = 0.8 * 0.2 * 0.166

# projection between the reward_prediction_error(rpe) and the ventral_tegmental_area (vta)
rpe_to_vta = Projection(pre=reward_prediction_error, post=ventral_tegmental_area, target="inh", synapse=DAPrediction)
rpe_to_vta.connect_all_to_all(weights=0.0)
rpe_to_vta.tau = 4500
rpe_to_vta.baseline_dopa = 0.15

# projection between the ventral_tegmental_area (vta) and the reward_prediction_error(rpe)
vta_to_rpe = Projection(pre=ventral_tegmental_area, post=reward_prediction_error, target="dopa")
vta_to_rpe.connect_all_to_all(weights=1.0)

# projection between the pedunculopontine tegmental nucleus (PPTN) and the ventral_tegmental_area (vta) to enable the participant reward
pptn_ventral_to_vta = Projection(pre=pptn_ventral, post=ventral_tegmental_area, target="exc")
pptn_ventral_to_vta.connect_one_to_one(weights=1.0)

# projection of the ventral_tegmental_area to different areas of the ventral loop
# to regulate the dopamin output encoded in the ventral_tegmental area from a separate
# group of cells with a constant input instead of an ever-changing one
# projection between the ventral_tegmental_area (vta) and the nacc_shell_d1
vta_to_nacc_shell_d1 = Projection(pre=ventral_tegmental_area, post=nacc_shell_d1, target="dopa")
vta_to_nacc_shell_d1.connect_all_to_all(weights=1.0)

# projection between the ventral_tegmental_area (vta) and the nacc_shell_d2
vta_to_nacc_shell_d2 = Projection(pre=ventral_tegmental_area, post=nacc_shell_d2, target="dopa")
vta_to_nacc_shell_d2.connect_all_to_all(weights=1.0)

# projection between the ventral_tegmental_area (vta) and the stn_ventral
vta_to_stn_ventral = Projection(pre=ventral_tegmental_area, post=stn_ventral, target="dopa")
vta_to_stn_ventral.connect_all_to_all(weights=1.0)

# projection between the ventral_tegmental_area (vta) and the vp_direct
vta_to_vp_direct = Projection(pre=ventral_tegmental_area, post=vp_direct, target="dopa")
vta_to_vp_direct.connect_all_to_all(weights=1.0)

# projection between the ventral_tegmental_area (vta) and the vp_indirect
vta_to_vp_indirect = Projection(pre=ventral_tegmental_area, post=vp_indirect, target="dopa")
vta_to_vp_indirect.connect_all_to_all(weights=1.0)

## feedback part #####################################################################################
# cortical feedback of the medial PFC to have a learning effect in the ventral pallidum
# projection of the medial PFC to the vp_direct and vp indirect through a cortical feedback function
cortical_feedback_ventral_to_cortical_feedback_ventral = Projection(pre=cortical_feedback_ventral, post=cortical_feedback_ventral, target="inh")
cortical_feedback_ventral_to_cortical_feedback_ventral.connect_all_to_all(weights=0.1)

# projection of the cortical_feedback to the vp_direct
cortical_feedback_ventral_to_vp_direct = Projection(pre=cortical_feedback_ventral, post=vp_direct, target="inh")
cortical_feedback_ventral_to_vp_direct.connect_one_to_one(weights=1.1)

# projection of the cortical feedback to the vp_indirect
cortical_feedback_ventral_to_vp_indirect = Projection(pre=cortical_feedback_ventral, post=vp_direct, target="inh")
cortical_feedback_ventral_to_vp_indirect.connect_one_to_one(weights=0.3)

######################################################################################################
### Projections of the working memory (handcrafted) ##################################################
######################################################################################################

# projection thalamus_ventral to the medial PFC of the working memory
thalamus_ventral_to_medial_pfc = Projection(pre=thalamus_ventral, post=medial_pfc, target="exc")
thalamus_ventral_to_medial_pfc.connect_one_to_one(weights=1.0)

# local projection within the medial PFC
medial_pfc_to_medial_pfc = Projection(pre=medial_pfc, post=medial_pfc, target="inh")
medial_pfc_to_medial_pfc.connect_all_to_all(weights=0.8)

# projection of the medial_PFC to the cortical_feedback_ventral
medial_pfc_to_cortical_feedback_ventral = Projection(pre=medial_pfc, post=cortical_feedback_ventral, target="exc")
medial_pfc_to_cortical_feedback_ventral.connect_one_to_one(weights=1.2)

# projection of the medial_PFC to nacc_core of the working memory
medial_pfc_to_nacc_core = Projection(pre=medial_pfc, post=nacc_core, target="exc")
medial_pfc_to_nacc_core.connect_one_to_one(weights=2.0)

# projection of the nacc_core to the substantia_nigra_pars_reticulata of the working memory
nacc_core_to_snr = Projection(pre=nacc_core, post=substantia_nigra_pars_reticulata, target="inh")
nacc_core_to_snr.connect_one_to_one(weights=1.0)

# projection of the substantia_nigra_pars_reticulata to the thalamus_ventral of the working memory
snr_to_thalamus_ventral = Projection(pre=substantia_nigra_pars_reticulata, post=thalamus_ventral, target="inh")
snr_to_thalamus_ventral.connect_one_to_one(weights=0.75)

######################################################################################################
### Projections of the dorsomedial loop (caudate) ####################################################
######################################################################################################

## direct pathway dorsomedial ########################################################################
# projection of the medial_PFC to the striatum_d1_dorsomedial of the dorsomedial loop as direct pathway
medial_pfc_to_striatum_d1_dorsomedial = Projection(pre=medial_pfc, post=striatum_d1_dorsomedial, target="exc", synapse=DAPostCovarianceNoThreshold_trace)
medial_pfc_to_striatum_d1_dorsomedial.connect_all_to_all(weights=Normal(0.5, 0.2))
medial_pfc_to_striatum_d1_dorsomedial.tau = 400
medial_pfc_to_striatum_d1_dorsomedial.regularization_threshold = 2.0
medial_pfc_to_striatum_d1_dorsomedial.tau_alpha = 5.0
medial_pfc_to_striatum_d1_dorsomedial.baseline_dopa = 0.8
medial_pfc_to_striatum_d1_dorsomedial.K_dip = 0.05
medial_pfc_to_striatum_d1_dorsomedial.K_burst = 1.0
medial_pfc_to_striatum_d1_dorsomedial.DA_type = 1
medial_pfc_to_striatum_d1_dorsomedial.threshold_pre = 0.2
medial_pfc_to_striatum_d1_dorsomedial.threshold_post = 0.0

# local inhibitory connection of the striatum_d1_dorsomedial
striatum_d1_dorsomedial_to_striatum_d1_dorsomedial = Projection(pre=striatum_d1_dorsomedial, post=striatum_d1_dorsomedial, target="inh")
striatum_d1_dorsomedial_to_striatum_d1_dorsomedial.connect_all_to_all(weights=0.2)

# projection of the striatum_d1_dorsomedial to the gpi_dorsomedial as direct pathway
striatum_d1_dorsomedial_to_gpi_dorsomedial = Projection(pre=striatum_d1_dorsomedial, post=gpi_dorsomedial, target="inh", synapse=DAPreCovariance_inhibitory_trace)
striatum_d1_dorsomedial_to_gpi_dorsomedial.connect_all_to_all(weights=Normal(0.1, 0.01))
striatum_d1_dorsomedial_to_gpi_dorsomedial.tau = 1600
striatum_d1_dorsomedial_to_gpi_dorsomedial.regularization_threshold = 2.25
striatum_d1_dorsomedial_to_gpi_dorsomedial.tau_alpha = 4.0
striatum_d1_dorsomedial_to_gpi_dorsomedial.baseline_dopa = 0.8
striatum_d1_dorsomedial_to_gpi_dorsomedial.K_dip = 0.9
striatum_d1_dorsomedial_to_gpi_dorsomedial.K_burst = 1.0
striatum_d1_dorsomedial_to_gpi_dorsomedial.DA_type = 1
striatum_d1_dorsomedial_to_gpi_dorsomedial.threshold_pre = 0.05
striatum_d1_dorsomedial_to_gpi_dorsomedial.threshold_post = 0.3
striatum_d1_dorsomedial_to_gpi_dorsomedial.negterm = 5.0

# local excitatory connections of the gpi
# split into two sections to enable learning of both stage choices at the same time
gpi_dorsomedial_to_gpi_dorsomedial = Projection(pre=gpi_dorsomedial, post=gpi_dorsomedial, target="exc", synapse=ReversedSynapse)
gpi_dorsomedial_to_gpi_dorsomedial.connect_all_to_all(weights=0.15)
gpi_dorsomedial_to_gpi_dorsomedial.reversal = 0.4

# local excitatory connection of the gpi used to train the decision of the first stage
gpi_dorsomedial_to_gpi_dorsomedial_12 = Projection(pre=gpi_dorsomedial[0:2], post=gpi_dorsomedial[0:2], target="exc", synapse=ReversedSynapse)
gpi_dorsomedial_to_gpi_dorsomedial_12.connect_all_to_all(weights=0.35)
gpi_dorsomedial_to_gpi_dorsomedial_12.reversal = 0.4

# local excitatory connection of the gpi used to train the decision of the second stage
gpi_dorsomedial_to_gpi_dorsomedial_34 = Projection(pre=gpi_dorsomedial[2:4], post=gpi_dorsomedial[2:4], target="exc", synapse=ReversedSynapse)
gpi_dorsomedial_to_gpi_dorsomedial_34.connect_all_to_all(weights=0.35)
gpi_dorsomedial_to_gpi_dorsomedial_34.reversal = 0.4

# local excitatory connection of the gpi used to train the decision of the second stage
gpi_dorsomedial_to_gpi_dorsomedial_56 = Projection(pre=gpi_dorsomedial[4:], post=gpi_dorsomedial[4:], target="exc", synapse=ReversedSynapse)
gpi_dorsomedial_to_gpi_dorsomedial_56.connect_all_to_all(weights=0.35)
gpi_dorsomedial_to_gpi_dorsomedial_56.reversal = 0.4

# projection of the gpi_dorsomedial to the thalamus_dorsomedial
gpi_dorsomedial_to_thalamus_dorsomedial = Projection(pre=gpi_dorsomedial, post=thalamus_dorsomedial, target="inh")
gpi_dorsomedial_to_thalamus_dorsomedial.connect_one_to_one(weights=1.7)

# local inhibitory connections of the thalamus
# split into two sections to enable learning of both stage choices at the same time
thalamus_dorsomedial_to_thalamus_dorsomedial = Projection(pre=thalamus_dorsomedial[0:2], post=thalamus_dorsomedial[0:2], target="inh")
thalamus_dorsomedial_to_thalamus_dorsomedial.connect_all_to_all(weights=0.05)

# local inhibitory connections of the thalamus used to train the decision of the first stage
thalamus_dorsomedial_to_thalamus_dorsomedial_12 = Projection(pre=thalamus_dorsomedial[0:2], post=thalamus_dorsomedial[0:2], target="inh")
thalamus_dorsomedial_to_thalamus_dorsomedial_12.connect_all_to_all(weights=0.15)

# local inhibitory connections of the thalamus used to train the decision of the second stage
thalamus_dorsomedial_to_thalamus_dorsomedial_34 = Projection(pre=thalamus_dorsomedial[2:4], post=thalamus_dorsomedial[2:4], target="inh")
thalamus_dorsomedial_to_thalamus_dorsomedial_34.connect_all_to_all(weights=0.15)

# local inhibitory connections of the thalamus used to train the decision of the second stage
thalamus_dorsomedial_to_thalamus_dorsomedial_56 = Projection(pre=thalamus_dorsomedial[4:6], post=thalamus_dorsomedial[4:6], target="inh")
thalamus_dorsomedial_to_thalamus_dorsomedial_56.connect_all_to_all(weights=0.15)

# projection of the thalamus_dorsomedial to the dorsolateral_PFC as connection between dorsomedial and dorsolateral loop
thalamus_dorsomedial_to_dorsolateral_pfc = Projection(pre=thalamus_dorsomedial, post=dorsolateral_pfc, target="exc")
thalamus_dorsomedial_to_dorsolateral_pfc.connect_one_to_one(weights=1.5)

## indirect pathway dorsomedial ######################################################################
# projection of the medial_PFC to the striatum_d2_dorsomedial of the dorsomedial loop as indirect pathway
medial_pfc_to_striatum_d2_dorsomedial = Projection(pre=medial_pfc, post=striatum_d2_dorsomedial, target="exc", synapse=DAPostCovarianceNoThreshold)
medial_pfc_to_striatum_d2_dorsomedial.connect_all_to_all(weights=Normal(0.12, 0.03))
medial_pfc_to_striatum_d2_dorsomedial.tau = 2000
medial_pfc_to_striatum_d2_dorsomedial.regularization_threshold = 1.5
medial_pfc_to_striatum_d2_dorsomedial.tau_alpha = 15.0
medial_pfc_to_striatum_d2_dorsomedial.baseline_dopa = 0.8
medial_pfc_to_striatum_d2_dorsomedial.K_dip = 0.2
medial_pfc_to_striatum_d2_dorsomedial.K_burst = 1.0
medial_pfc_to_striatum_d2_dorsomedial.DA_type = -1
medial_pfc_to_striatum_d2_dorsomedial.threshold_pre = 0.05
medial_pfc_to_striatum_d2_dorsomedial.threshold_post = 0.05

# local inhibitory connection of the striatum_d2_dorsomedial
striatum_d2_dorsomedial_to_striatum_d2_dorsomedial = Projection(pre=striatum_d2_dorsomedial, post=striatum_d2_dorsomedial, target="inh")
striatum_d2_dorsomedial_to_striatum_d2_dorsomedial.connect_all_to_all(weights=(0.1 / 6))

# projection of the striatum_d2_dorsomedial to the gpe_dorsomedial as indirect pathway
striatum_d2_dorsomedial_to_gpe_dorsomedial = Projection(pre=striatum_d2_dorsomedial, post=gpe_dorsomedial, target="inh", synapse=DAPreCovariance_inhibitory)
striatum_d2_dorsomedial_to_gpe_dorsomedial.connect_all_to_all(weights=0.01)
striatum_d2_dorsomedial_to_gpe_dorsomedial.tau = 2500
striatum_d2_dorsomedial_to_gpe_dorsomedial.regularization_threshold = 1.5
striatum_d2_dorsomedial_to_gpe_dorsomedial.tau_alpha = 20.0
striatum_d2_dorsomedial_to_gpe_dorsomedial.baseline_dopa = 0.8
striatum_d2_dorsomedial_to_gpe_dorsomedial.K_dip = 0.1
striatum_d2_dorsomedial_to_gpe_dorsomedial.K_burst = 1.2
striatum_d2_dorsomedial_to_gpe_dorsomedial.threshold_pre = 0.1
striatum_d2_dorsomedial_to_gpe_dorsomedial.threshold_post = 0.0
striatum_d2_dorsomedial_to_gpe_dorsomedial.DA_type = -1

# projection of the gpe_dorsomedial to the gpi_dorsomedial as indirect pathway
gpe_dorsomedial_to_gpi_dorsomedial = Projection(pre=gpe_dorsomedial, post=gpi_dorsomedial, target="inh")
gpe_dorsomedial_to_gpi_dorsomedial.connect_one_to_one(weights=1.0)

## hyperdirect pathway dorsomedial ###################################################################
# projection of the visual_input to the stn_dorsomedial as the hyperdirect pathway
# for-loop as idea to fixate connections between separate columns of the visual_input with the respective neuron of the stn_dorsomedial
visual_input_to_stn_dorsomedial = []
for i in range(3):
    visual_input_to_stn_dorsomedial.append(Projection(pre=visual_input[:, i], post=stn_dorsomedial[i], target="exc"))
    visual_input_to_stn_dorsomedial[i].connect_all_to_all(weights=0.25)

# projection of the stn_dorsomedial to the gpi_dorsomedial as hyperdirect pathway
# separate fixed connections to the gpi to support the decision process of the second loop
stn_dorsomedial_to_gpi_dorsomedial_neuron_1 = Projection(pre=stn_dorsomedial[0], post=gpi_dorsomedial[:2], target="exc")
stn_dorsomedial_to_gpi_dorsomedial_neuron_1.connect_all_to_all(weights=1.7)
stn_dorsomedial_to_gpi_dorsomedial_neuron_2_1 = Projection(pre=stn_dorsomedial[1], post=gpi_dorsomedial[2:4], target="exc")
stn_dorsomedial_to_gpi_dorsomedial_neuron_2_1.connect_all_to_all(weights=1.7)
stn_dorsomedial_to_gpi_dorsomedial_neuron_2_2 = Projection(pre=stn_dorsomedial[2], post=gpi_dorsomedial[4:], target="exc")
stn_dorsomedial_to_gpi_dorsomedial_neuron_2_2.connect_all_to_all(weights=1.7)


## learning part #####################################################################################
# projection of the substantia_nigra_pars_compacta to different areas of the dorsomedial loop
# to regulate the dopamin output encoded in the substantia_nigra_pars_compacta from a separate
# group of cells with a constant input instead of an ever-changing one
# projection of the pptn_dorsomedial to the substantia_nigra_pars_compacta and the medial_PFC
pptn_dorsomedial_to_snc = Projection(pre=pptn_dorsomedial, post=substantia_nigra_pars_compacta, target="exc")
pptn_dorsomedial_to_snc.connect_one_to_one(weights=2.0)

pptn_dorsomedial_to_mpfc = Projection(pre=pptn_dorsomedial, post=medial_pfc, target="exc")
pptn_dorsomedial_to_mpfc.connect_one_to_one(weights=1.0)

# projection of the striatum_d1_dorsomedial to the substantia_nigra_pars_compacta
striatum_d1_dorsomedial_to_snc = Projection(pre=striatum_d1_dorsomedial, post=substantia_nigra_pars_compacta, target="inh", synapse=DAPrediction)
striatum_d1_dorsomedial_to_snc.connect_all_to_all(weights=0.0)
striatum_d1_dorsomedial_to_snc.tau = 3000

# projection of the substantia_nigra_pars_compacta to the striatum_d1_dorsomedial
snc_to_striatum_d1_dorsomedial = Projection(pre=substantia_nigra_pars_compacta, post=striatum_d1_dorsomedial, target="dopa")
snc_to_striatum_d1_dorsomedial.connect_all_to_all(weights=1.0)

# projection of the substantia_nigra_pars_compacta to the striatum_d2_dorsomedial
snc_to_striatum_d2_dorsomedial = Projection(pre=substantia_nigra_pars_compacta, post=striatum_d2_dorsomedial, target="dopa")
snc_to_striatum_d2_dorsomedial.connect_all_to_all(weights=1.0)

# projection of the substantia_nigra_pars_compacta to the stn_dorsomedial
snc_to_stn_dorsomedial = Projection(pre=substantia_nigra_pars_compacta, post=stn_dorsomedial, target="dopa")
snc_to_stn_dorsomedial.connect_all_to_all(weights=1.0)

# projection of the substantia_nigra_pars_compacta to the gpi_dorsomedial
snc_to_gpi_dorsomedial = Projection(pre=substantia_nigra_pars_compacta, post=gpi_dorsomedial, target="dopa")
snc_to_gpi_dorsomedial.connect_all_to_all(weights=1.0)

# projection of the substantia_nigra_pars_compacta to the gpe_dorsomedial
snc_to_gpe_dorsomedial = Projection(pre=substantia_nigra_pars_compacta, post=gpe_dorsomedial, target="dopa")
snc_to_gpe_dorsomedial.connect_all_to_all(weights=1.0)

## feedback part #####################################################################################
# cortical feedback of the dorsolateral PFC to have a learning effect in the globus pallidus
# projection of the dorsolateral PFC to the gpi and gpe through a cortical feedback function
dorsolateral_pfc_to_cortical_feedback_dorsomedial = Projection(pre=dorsolateral_pfc, post=cortical_feedback_dorsomedial, target="exc")
dorsolateral_pfc_to_cortical_feedback_dorsomedial.connect_one_to_one(weights=1.2)

dorsolateral_pfc_to_dorsolateral_pfc = Projection(pre=dorsolateral_pfc, post=dorsolateral_pfc, target="inh")
dorsolateral_pfc_to_dorsolateral_pfc.connect_all_to_all(weights=0.2)

dorsolateral_pfc_to_dorsolateral_pfc_12 = Projection(pre=dorsolateral_pfc[0:2], post=dorsolateral_pfc[0:2], target="inh")
dorsolateral_pfc_to_dorsolateral_pfc_12.connect_all_to_all(weights=0.5)

dorsolateral_pfc_to_dorsolateral_pfc_34 = Projection(pre=dorsolateral_pfc[2:4], post=dorsolateral_pfc[2:4], target="inh")
dorsolateral_pfc_to_dorsolateral_pfc_34.connect_all_to_all(weights=0.5)

dorsolateral_pfc_to_dorsolateral_pfc_56 = Projection(pre=dorsolateral_pfc[4:6], post=dorsolateral_pfc[4:6], target="inh")
dorsolateral_pfc_to_dorsolateral_pfc_56.connect_all_to_all(weights=0.5)

cortical_feedback_dorsomedial_to_cortical_feedback_dorsomedial = Projection(pre=cortical_feedback_dorsomedial, post=cortical_feedback_dorsomedial, target="inh")
cortical_feedback_dorsomedial_to_cortical_feedback_dorsomedial.connect_all_to_all(weights=(0.1 / 6))

# projection of the cortical_feedback_dorsomedial to the gpi_dorsomedial
cortical_feedback_dorsomedial_to_gpi = Projection(pre=cortical_feedback_dorsomedial, post=gpi_dorsomedial, target="inh")
cortical_feedback_dorsomedial_to_gpi.connect_one_to_one(weights=0.8)

# projection of the cortical_feedback_dorsomedial to the gpe_dorsomedial
cortical_feedback_dorsomedial_to_gpe = Projection(pre=cortical_feedback_dorsomedial, post=gpe_dorsomedial, target="inh")
cortical_feedback_dorsomedial_to_gpe.connect_one_to_one(weights=0.3)

######################################################################################################
### Projections of the dorsolateral loop (putamen) ###################################################
######################################################################################################

## direct pathway dorsolateral ########################################################################
# projection of the dorsolateral_PFC to the striatum_d1_dorsolateral of the dorsolateral loop as direct pathway
# all connections are fixed, as the dorsolateral loop is just for action execution, therefore the decision is made before
dorsolateral_pfc_to_striatum_d1_dorsolateral = Projection(pre=dorsolateral_pfc, post=striatum_d1_dorsolateral, target="exc")
dorsolateral_pfc_to_striatum_d1_dorsolateral.connect_one_to_one(weights=1.0)

# projection of the striatum_d1_dorsolateral to the gpi_dorsolateral
striatum_d1_dorsolateral_to_gpi_dorsolateral_1 = Projection(pre=striatum_d1_dorsolateral[0], post=gpi_dorsolateral[0], target="inh")
striatum_d1_dorsolateral_to_gpi_dorsolateral_1.connect_one_to_one(weights=1.0)

striatum_d1_dorsolateral_to_gpi_dorsolateral_2 = Projection(pre=striatum_d1_dorsolateral[1], post=gpi_dorsolateral[0], target="inh")
striatum_d1_dorsolateral_to_gpi_dorsolateral_2.connect_one_to_one(weights=1.0)

striatum_d1_dorsolateral_to_gpi_dorsolateral_3 = Projection(pre=striatum_d1_dorsolateral[2], post=gpi_dorsolateral[1], target="inh")
striatum_d1_dorsolateral_to_gpi_dorsolateral_3.connect_one_to_one(weights=1.0)

# projection of the striatum_d1_dorsolateral to the gpi_dorsolateral
striatum_d1_dorsolateral_to_gpi_dorsolateral_4 = Projection(pre=striatum_d1_dorsolateral[3], post=gpi_dorsolateral[1], target="inh")
striatum_d1_dorsolateral_to_gpi_dorsolateral_4.connect_one_to_one(weights=1.0)

striatum_d1_dorsolateral_to_gpi_dorsolateral_5 = Projection(pre=striatum_d1_dorsolateral[4], post=gpi_dorsolateral[2], target="inh")
striatum_d1_dorsolateral_to_gpi_dorsolateral_5.connect_one_to_one(weights=1.0)

striatum_d1_dorsolateral_to_gpi_dorsolateral_6 = Projection(pre=striatum_d1_dorsolateral[5], post=gpi_dorsolateral[2], target="inh")
striatum_d1_dorsolateral_to_gpi_dorsolateral_6.connect_one_to_one(weights=1.0)

# projection of the gpi_dorsolateral to the thalamus_dorsolateral
gpi_dorsolateral_to_thalamus_dorsolateral = Projection(pre=gpi_dorsolateral, post=thalamus_dorsolateral,target="inh")
gpi_dorsolateral_to_thalamus_dorsolateral.connect_one_to_one(weights=1.0)

# projection of the thalamus_dorsolateral to the premotor
thalamus_dorsolateral_to_premotor = Projection(pre=thalamus_dorsolateral, post=premotor, target="exc")
thalamus_dorsolateral_to_premotor.connect_one_to_one(weights=1.0)

# local inhibitory connections of the premotor
premotor_to_premotor = Projection(pre=premotor, post=premotor, target="inh")
premotor_to_premotor.connect_all_to_all(weights=0.4)

# net1 = Network(everything=True)
# net1.compile()
# compile()

class Basal_Agent:
    def __init__(self):


        # execution of a predefined model, in this case the basal_ganglia_V1.0.py
        #exec(open("basal_ganglia_2011_V1.0.py").read())
        self.net = Network(everything=True)
        self.net.compile()

        # self.state_rep = Action_State_RL()

        self.hippocampus = self.net.get(hippocampus)
        self.hippocampus_activity = self.net.get(hippocampus_activity)
        self.reward_prediction_error = self.net.get(reward_prediction_error)
        self.nacc_shell_d1 = self.net.get(nacc_shell_d1)
        self.nacc_shell_d2 = self.net.get(nacc_shell_d2)
        self.stn_ventral = self.net.get(stn_ventral)
        self.vp_direct = self.net.get(vp_direct)
        self.vp_indirect = self.net.get(vp_indirect)
        self.ventral_tegmental_area = self.net.get(ventral_tegmental_area)
        self.pptn_ventral = self.net.get(pptn_ventral)
        self.cortical_feedback_ventral = self.net.get(cortical_feedback_ventral)

        self.thalamus_ventral = self.net.get(thalamus_ventral)
        self.medial_pfc = self.net.get(medial_pfc)
        self.nacc_core = self.net.get(nacc_core)
        self.substantia_nigra_pars_reticulata = self.net.get(substantia_nigra_pars_reticulata)

        self.visual_input = self.net.get(visual_input)
        self.striatum_d1_dorsomedial = self.net.get(striatum_d1_dorsomedial)
        self.striatum_d2_dorsomedial = self.net.get(striatum_d2_dorsomedial)
        self.stn_dorsomedial = self.net.get(stn_dorsomedial)
        self.gpi_dorsomedial = self.net.get(gpi_dorsomedial)
        self.gpe_dorsomedial = self.net.get(gpe_dorsomedial)
        self.substantia_nigra_pars_compacta = self.net.get(substantia_nigra_pars_compacta)
        self.pptn_dorsomedial = self.net.get(pptn_dorsomedial)
        self.cortical_feedback_dorsomedial = self.net.get(cortical_feedback_dorsomedial)
        self.thalamus_dorsomedial = self.net.get(thalamus_dorsomedial)

        self.dorsolateral_pfc = self.net.get(dorsolateral_pfc)
        self.striatum_d1_dorsolateral = self.net.get(striatum_d1_dorsolateral)
        self.gpi_dorsolateral = self.net.get(gpi_dorsolateral)
        self.thalamus_dorsolateral = self.net.get(thalamus_dorsolateral)
        self.premotor = self.net.get(premotor)

    def run_network(self, input):

        # enables learning for all projections
        self.net.enable_learning()

        # the generator creates a random number based on the seed value, by default the current system time to create the equal start for every trial
        # random.seed(10)

        # reset everything at the start of the trial
        # ventral loop
        self.hippocampus.baseline = 0.0
        self.hippocampus.r = 0.0
        self.thalamus_ventral.baseline = 0.0
        self.thalamus_ventral.r = 0.0
        self.medial_pfc.baseline = 0.0
        self.medial_pfc.r = 0.0
        self.pptn_ventral.baseline = 0.0

        # dorsomedial loop
        self.visual_input.baseline = 0.0
        self.striatum_d1_dorsomedial.r = 0.0
        self.striatum_d2_dorsomedial.r = 0.0
        self.gpi_dorsomedial.r = 0.0
        self.gpe_dorsomedial.r = 0.0
        self.thalamus_dorsomedial.baseline = 0.0
        self.thalamus_dorsomedial.r = 0.0
        self.dorsolateral_pfc.baseline = 0.0
        self.dorsolateral_pfc.r = 0.0
        self.cortical_feedback_dorsomedial.learning = 1.0
        self.cortical_feedback_dorsomedial.r = 0.0
        self.pptn_dorsomedial.baseline = 0.0
        self.pptn_dorsomedial.r = 0.0

        # simulation of the model for 2000ms to realize the reset
        self.net.simulate(2000)

        # further changes of baseline values to finish the reset
        self.thalamus_ventral.baseline = 1.15
        self.thalamus_dorsomedial.baseline = 0.95

        # simulation of the model for 300ms with recently made changes
        self.net.simulate(300)

        """
            Input of hippocampus_model results into the model and check if the sensor recognizes a wall
        """
        for i in range(len(input)):
            value = input[i]
            if value > 0:
                hippocampus[:, i].baseline = 0.5
            else:
                hippocampus[:, i].baseline = 0.0
        sensor = self.state_rep.check_wall()

        if sensor == True:
           self.visual_input[:, 1].baseline = 1.5
        else:
           self.visual_input.baseline = 0.0


        self.net.simulate(300)

        # after the presentation of the stimuli, complete reset of the hippocampus and the visual_input
        self.hippocampus.baseline = 0.0
        # self.visual_input.baseline = 0.0

        self.net.simulate(300)

        # action that is used to move the robot
        eps = 0.000001
        premotor_objective = self.premotor.r
        action = random.choices([0, 1, 2], weights=premotor_objective + eps, k=1)[0]

        predictive_pfc_objective = self.medial_pfc.r
        predicted_action = random.choices([0, 0, 1, 1, 2, 2], weights=predictive_pfc_objective + eps, k=1)[0]

        print("predicted_action: ", predicted_action)

        achieved_sequence_1 = 2 * action - 2
        achieved_sequence_2 = 2 * action - 1

        ### learning part
        # integration of the realized objective into the cortex
        # with very high pfc activities (otherwise unlearning because of regularization with alpha

        for i in range(len(input)):
            value = input[i]
            if value > 0:
                hippocampus[:, i].baseline = 0.5
            else:
                hippocampus[:, i].baseline = 0.0

        self.visual_input.baseline = 0.0

        self.pptn_dorsomedial[achieved_sequence_1].baseline = 2.0
        self.pptn_dorsomedial[achieved_sequence_2].baseline = 2.0
        self.dorsolateral_pfc[2 * action - 1].baseline = 2.0
        self.dorsolateral_pfc[2 * action - 2].baseline = 2.0

        # short simulation of the model to integrate the new set baselines
        self.net.simulate(100)

        # reduction of the feedback strength
        self.dorsolateral_pfc[2 * action - 1].baseline = 1.5
        self.dorsolateral_pfc[2 * action - 2].baseline = 1.5
        self.cortical_feedback_ventral.learning = 1.0
        self.pptn_dorsomedial[achieved_sequence_1].baseline = 0.2
        self.pptn_dorsomedial[achieved_sequence_2].baseline = 0.2

        # short simulation for cortical integration
        self.net.simulate(300)


        # reward of the task:
        # visual_input sets a goal signal, if reached -> reward, else not

        if action == predicted_action:
            pptn_ventral.baseline = 1.0
        else:
            pptn_ventral.baseline = 0.0


        # learning period by firing of VTA and SNc
        self.ventral_tegmental_area.firing = 1.0
        self.substantia_nigra_pars_compacta.firing = 1.0

        # short simulation that the learning takes effect
        self.net.simulate(100)

        # reset of the in the learning period involved parts
        self.ventral_tegmental_area.firing = 0.0
        self.pptn_ventral.baseline = 0.0
        self.cortical_feedback_ventral.learning = 0.0
        self.substantia_nigra_pars_compacta.firing = 0.0

        print("selected_action: ", action)

        return action

