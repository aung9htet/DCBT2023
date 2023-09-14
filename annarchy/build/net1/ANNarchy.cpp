
#include "ANNarchy.h"



/*
 * Internal data
 *
 */
double dt;
long int t;
std::vector<std::mt19937> rng;

// Custom constants


// Populations
PopStruct0 pop0;
PopStruct1 pop1;
PopStruct2 pop2;
PopStruct3 pop3;
PopStruct4 pop4;
PopStruct5 pop5;
PopStruct6 pop6;
PopStruct7 pop7;
PopStruct8 pop8;
PopStruct9 pop9;
PopStruct10 pop10;
PopStruct11 pop11;
PopStruct12 pop12;
PopStruct13 pop13;
PopStruct14 pop14;
PopStruct15 pop15;
PopStruct16 pop16;
PopStruct17 pop17;
PopStruct18 pop18;
PopStruct19 pop19;
PopStruct20 pop20;
PopStruct21 pop21;
PopStruct22 pop22;
PopStruct23 pop23;
PopStruct24 pop24;
PopStruct25 pop25;
PopStruct26 pop26;
PopStruct27 pop27;
PopStruct28 pop28;
PopStruct29 pop29;


// Projections
ProjStruct0 proj0;
ProjStruct1 proj1;
ProjStruct2 proj2;
ProjStruct3 proj3;
ProjStruct4 proj4;
ProjStruct5 proj5;
ProjStruct6 proj6;
ProjStruct7 proj7;
ProjStruct8 proj8;
ProjStruct9 proj9;
ProjStruct10 proj10;
ProjStruct11 proj11;
ProjStruct12 proj12;
ProjStruct13 proj13;
ProjStruct14 proj14;
ProjStruct15 proj15;
ProjStruct16 proj16;
ProjStruct17 proj17;
ProjStruct18 proj18;
ProjStruct19 proj19;
ProjStruct20 proj20;
ProjStruct21 proj21;
ProjStruct22 proj22;
ProjStruct23 proj23;
ProjStruct24 proj24;
ProjStruct25 proj25;
ProjStruct26 proj26;
ProjStruct27 proj27;
ProjStruct28 proj28;
ProjStruct29 proj29;
ProjStruct30 proj30;
ProjStruct31 proj31;
ProjStruct32 proj32;
ProjStruct33 proj33;
ProjStruct34 proj34;
ProjStruct35 proj35;
ProjStruct36 proj36;
ProjStruct37 proj37;
ProjStruct38 proj38;
ProjStruct39 proj39;
ProjStruct40 proj40;
ProjStruct41 proj41;
ProjStruct42 proj42;
ProjStruct43 proj43;
ProjStruct44 proj44;
ProjStruct45 proj45;
ProjStruct46 proj46;
ProjStruct47 proj47;
ProjStruct48 proj48;
ProjStruct49 proj49;
ProjStruct50 proj50;
ProjStruct51 proj51;
ProjStruct52 proj52;
ProjStruct53 proj53;
ProjStruct54 proj54;
ProjStruct55 proj55;
ProjStruct56 proj56;
ProjStruct57 proj57;
ProjStruct58 proj58;
ProjStruct59 proj59;
ProjStruct60 proj60;
ProjStruct61 proj61;
ProjStruct62 proj62;
ProjStruct63 proj63;
ProjStruct64 proj64;
ProjStruct65 proj65;
ProjStruct66 proj66;
ProjStruct67 proj67;
ProjStruct68 proj68;
ProjStruct69 proj69;
ProjStruct70 proj70;
ProjStruct71 proj71;
ProjStruct72 proj72;
ProjStruct73 proj73;
ProjStruct74 proj74;
ProjStruct75 proj75;
ProjStruct76 proj76;
ProjStruct77 proj77;
ProjStruct78 proj78;
ProjStruct79 proj79;
ProjStruct80 proj80;
ProjStruct81 proj81;
ProjStruct82 proj82;
ProjStruct83 proj83;
ProjStruct84 proj84;


// Global operations

// Computes the mean value of an array
double mean_value(const double* array, int n)
{
    double sum = array[0];
    for(int i=1; i<n; i++)
    {
        sum += array[i];
    }
    return sum/(double)n;
}


/*
 * Recorders
 */
std::vector<Monitor*> recorders;
int addRecorder(Monitor* recorder){
    int found = -1;

    for (unsigned int i=0; i<recorders.size(); i++) {
        if (recorders[i] == nullptr) {
            found = i;
            break;
        }
    }

    if (found != -1) {
        // fill a previously cleared slot
        recorders[found] = recorder;
        return found;
    } else {
        recorders.push_back(recorder);
        return recorders.size() - 1;
    }
}
Monitor* getRecorder(int id) {
    if (id < recorders.size())
        return recorders[id];
    else
        return nullptr;
}
void removeRecorder(Monitor* recorder){
    for (unsigned int i=0; i<recorders.size(); i++){
        if(recorders[i] == recorder){
            recorders[i] = nullptr;
            break;
        }
    }
}

/*
 *  Simulation methods
 */
// Simulate a single step
void singleStep()
{


    ////////////////////////////////
    // Presynaptic events
    ////////////////////////////////


    // pop1: hippocampus_activity
    if (pop1._active)
        std::fill(pop1._sum_exc.begin(), pop1._sum_exc.end(), static_cast<double>(0.0) );

    // pop2: reward_prediction_error
    if (pop2._active)
        std::fill(pop2._sum_dopa.begin(), pop2._sum_dopa.end(), static_cast<double>(0.0) );

    // pop2: reward_prediction_error
    if (pop2._active)
        std::fill(pop2._sum_exc.begin(), pop2._sum_exc.end(), static_cast<double>(0.0) );

    // pop2: reward_prediction_error
    if (pop2._active)
        std::fill(pop2._sum_inh.begin(), pop2._sum_inh.end(), static_cast<double>(0.0) );

    // pop3: nacc_shell_d1
    if (pop3._active)
        std::fill(pop3._sum_dopa.begin(), pop3._sum_dopa.end(), static_cast<double>(0.0) );

    // pop3: nacc_shell_d1
    if (pop3._active)
        std::fill(pop3._sum_exc.begin(), pop3._sum_exc.end(), static_cast<double>(0.0) );

    // pop3: nacc_shell_d1
    if (pop3._active)
        std::fill(pop3._sum_inh.begin(), pop3._sum_inh.end(), static_cast<double>(0.0) );

    // pop4: nacc_shell_d2
    if (pop4._active)
        std::fill(pop4._sum_dopa.begin(), pop4._sum_dopa.end(), static_cast<double>(0.0) );

    // pop4: nacc_shell_d2
    if (pop4._active)
        std::fill(pop4._sum_exc.begin(), pop4._sum_exc.end(), static_cast<double>(0.0) );

    // pop4: nacc_shell_d2
    if (pop4._active)
        std::fill(pop4._sum_inh.begin(), pop4._sum_inh.end(), static_cast<double>(0.0) );

    // pop5: stn_ventral
    if (pop5._active)
        std::fill(pop5._sum_dopa.begin(), pop5._sum_dopa.end(), static_cast<double>(0.0) );

    // pop5: stn_ventral
    if (pop5._active)
        std::fill(pop5._sum_exc.begin(), pop5._sum_exc.end(), static_cast<double>(0.0) );

    // pop5: stn_ventral
    if (pop5._active)
        std::fill(pop5._sum_inh.begin(), pop5._sum_inh.end(), static_cast<double>(0.0) );

    // pop6: vp_direct
    if (pop6._active)
        std::fill(pop6._sum_dopa.begin(), pop6._sum_dopa.end(), static_cast<double>(0.0) );

    // pop6: vp_direct
    if (pop6._active)
        std::fill(pop6._sum_exc.begin(), pop6._sum_exc.end(), static_cast<double>(0.0) );

    // pop6: vp_direct
    if (pop6._active)
        std::fill(pop6._sum_inh.begin(), pop6._sum_inh.end(), static_cast<double>(0.0) );

    // pop7: vp_indirect
    if (pop7._active)
        std::fill(pop7._sum_dopa.begin(), pop7._sum_dopa.end(), static_cast<double>(0.0) );

    // pop7: vp_indirect
    if (pop7._active)
        std::fill(pop7._sum_inh.begin(), pop7._sum_inh.end(), static_cast<double>(0.0) );

    // pop8: ventral_tegmental_area
    if (pop8._active)
        std::fill(pop8._sum_exc.begin(), pop8._sum_exc.end(), static_cast<double>(0.0) );

    // pop8: ventral_tegmental_area
    if (pop8._active)
        std::fill(pop8._sum_inh.begin(), pop8._sum_inh.end(), static_cast<double>(0.0) );

    // pop10: cortical_feedback_ventral
    if (pop10._active)
        std::fill(pop10._sum_exc.begin(), pop10._sum_exc.end(), static_cast<double>(0.0) );

    // pop10: cortical_feedback_ventral
    if (pop10._active)
        std::fill(pop10._sum_inh.begin(), pop10._sum_inh.end(), static_cast<double>(0.0) );

    // pop11: thalamus_ventral
    if (pop11._active)
        std::fill(pop11._sum_inh.begin(), pop11._sum_inh.end(), static_cast<double>(0.0) );

    // pop12: medial_pfc
    if (pop12._active)
        std::fill(pop12._sum_exc.begin(), pop12._sum_exc.end(), static_cast<double>(0.0) );

    // pop12: medial_pfc
    if (pop12._active)
        std::fill(pop12._sum_inh.begin(), pop12._sum_inh.end(), static_cast<double>(0.0) );

    // pop13: nacc_core
    if (pop13._active)
        std::fill(pop13._sum_exc.begin(), pop13._sum_exc.end(), static_cast<double>(0.0) );

    // pop14: substantia_nigra_pars_reticulata
    if (pop14._active)
        std::fill(pop14._sum_inh.begin(), pop14._sum_inh.end(), static_cast<double>(0.0) );

    // pop16: striatum_d1_dorsomedial
    if (pop16._active)
        std::fill(pop16._sum_dopa.begin(), pop16._sum_dopa.end(), static_cast<double>(0.0) );

    // pop16: striatum_d1_dorsomedial
    if (pop16._active)
        std::fill(pop16._sum_exc.begin(), pop16._sum_exc.end(), static_cast<double>(0.0) );

    // pop16: striatum_d1_dorsomedial
    if (pop16._active)
        std::fill(pop16._sum_inh.begin(), pop16._sum_inh.end(), static_cast<double>(0.0) );

    // pop17: striatum_d2_dorsomedial
    if (pop17._active)
        std::fill(pop17._sum_dopa.begin(), pop17._sum_dopa.end(), static_cast<double>(0.0) );

    // pop17: striatum_d2_dorsomedial
    if (pop17._active)
        std::fill(pop17._sum_exc.begin(), pop17._sum_exc.end(), static_cast<double>(0.0) );

    // pop17: striatum_d2_dorsomedial
    if (pop17._active)
        std::fill(pop17._sum_inh.begin(), pop17._sum_inh.end(), static_cast<double>(0.0) );

    // pop18: stn_dorsomedial
    if (pop18._active)
        std::fill(pop18._sum_dopa.begin(), pop18._sum_dopa.end(), static_cast<double>(0.0) );

    // pop18: stn_dorsomedial
    if (pop18._active)
        std::fill(pop18._sum_exc.begin(), pop18._sum_exc.end(), static_cast<double>(0.0) );

    // pop19: gpi_dorsomedial
    if (pop19._active)
        std::fill(pop19._sum_dopa.begin(), pop19._sum_dopa.end(), static_cast<double>(0.0) );

    // pop19: gpi_dorsomedial
    if (pop19._active)
        std::fill(pop19._sum_exc.begin(), pop19._sum_exc.end(), static_cast<double>(0.0) );

    // pop19: gpi_dorsomedial
    if (pop19._active)
        std::fill(pop19._sum_inh.begin(), pop19._sum_inh.end(), static_cast<double>(0.0) );

    // pop20: gpe_dorsomedial
    if (pop20._active)
        std::fill(pop20._sum_dopa.begin(), pop20._sum_dopa.end(), static_cast<double>(0.0) );

    // pop20: gpe_dorsomedial
    if (pop20._active)
        std::fill(pop20._sum_inh.begin(), pop20._sum_inh.end(), static_cast<double>(0.0) );

    // pop21: substantia_nigra_pars_compacta
    if (pop21._active)
        std::fill(pop21._sum_exc.begin(), pop21._sum_exc.end(), static_cast<double>(0.0) );

    // pop21: substantia_nigra_pars_compacta
    if (pop21._active)
        std::fill(pop21._sum_inh.begin(), pop21._sum_inh.end(), static_cast<double>(0.0) );

    // pop23: cortical_feedback_dorsomedial
    if (pop23._active)
        std::fill(pop23._sum_exc.begin(), pop23._sum_exc.end(), static_cast<double>(0.0) );

    // pop23: cortical_feedback_dorsomedial
    if (pop23._active)
        std::fill(pop23._sum_inh.begin(), pop23._sum_inh.end(), static_cast<double>(0.0) );

    // pop24: thalamus_dorsomedial
    if (pop24._active)
        std::fill(pop24._sum_inh.begin(), pop24._sum_inh.end(), static_cast<double>(0.0) );

    // pop25: dorsolateral_pfc
    if (pop25._active)
        std::fill(pop25._sum_exc.begin(), pop25._sum_exc.end(), static_cast<double>(0.0) );

    // pop25: dorsolateral_pfc
    if (pop25._active)
        std::fill(pop25._sum_inh.begin(), pop25._sum_inh.end(), static_cast<double>(0.0) );

    // pop26: striatum_d1_dorsolateral
    if (pop26._active)
        std::fill(pop26._sum_exc.begin(), pop26._sum_exc.end(), static_cast<double>(0.0) );

    // pop27: gpi_dorsolateral
    if (pop27._active)
        std::fill(pop27._sum_inh.begin(), pop27._sum_inh.end(), static_cast<double>(0.0) );

    // pop28: thalamus_dorsolateral
    if (pop28._active)
        std::fill(pop28._sum_inh.begin(), pop28._sum_inh.end(), static_cast<double>(0.0) );

    // pop29: premotor
    if (pop29._active)
        std::fill(pop29._sum_exc.begin(), pop29._sum_exc.end(), static_cast<double>(0.0) );

    // pop29: premotor
    if (pop29._active)
        std::fill(pop29._sum_inh.begin(), pop29._sum_inh.end(), static_cast<double>(0.0) );

#ifdef _TRACE_SIMULATION_STEPS
    std::cout << "Update psp/conductances ..." << std::endl;
#endif
	proj0.compute_psp();
	proj1.compute_psp();
	proj2.compute_psp();
	proj3.compute_psp();
	proj4.compute_psp();
	proj5.compute_psp();
	proj6.compute_psp();
	proj7.compute_psp();
	proj8.compute_psp();
	proj9.compute_psp();
	proj10.compute_psp();
	proj11.compute_psp();
	proj12.compute_psp();
	proj13.compute_psp();
	proj14.compute_psp();
	proj15.compute_psp();
	proj16.compute_psp();
	proj17.compute_psp();
	proj18.compute_psp();
	proj19.compute_psp();
	proj20.compute_psp();
	proj21.compute_psp();
	proj22.compute_psp();
	proj23.compute_psp();
	proj24.compute_psp();
	proj25.compute_psp();
	proj26.compute_psp();
	proj27.compute_psp();
	proj28.compute_psp();
	proj29.compute_psp();
	proj30.compute_psp();
	proj31.compute_psp();
	proj32.compute_psp();
	proj33.compute_psp();
	proj34.compute_psp();
	proj35.compute_psp();
	proj36.compute_psp();
	proj37.compute_psp();
	proj38.compute_psp();
	proj39.compute_psp();
	proj40.compute_psp();
	proj41.compute_psp();
	proj42.compute_psp();
	proj43.compute_psp();
	proj44.compute_psp();
	proj45.compute_psp();
	proj46.compute_psp();
	proj47.compute_psp();
	proj48.compute_psp();
	proj49.compute_psp();
	proj50.compute_psp();
	proj51.compute_psp();
	proj52.compute_psp();
	proj53.compute_psp();
	proj54.compute_psp();
	proj55.compute_psp();
	proj56.compute_psp();
	proj57.compute_psp();
	proj58.compute_psp();
	proj59.compute_psp();
	proj60.compute_psp();
	proj61.compute_psp();
	proj62.compute_psp();
	proj63.compute_psp();
	proj64.compute_psp();
	proj65.compute_psp();
	proj66.compute_psp();
	proj67.compute_psp();
	proj68.compute_psp();
	proj69.compute_psp();
	proj70.compute_psp();
	proj71.compute_psp();
	proj72.compute_psp();
	proj73.compute_psp();
	proj74.compute_psp();
	proj75.compute_psp();
	proj76.compute_psp();
	proj77.compute_psp();
	proj78.compute_psp();
	proj79.compute_psp();
	proj80.compute_psp();
	proj81.compute_psp();
	proj82.compute_psp();
	proj83.compute_psp();
	proj84.compute_psp();



    ////////////////////////////////
    // Recording target variables
    ////////////////////////////////
#ifdef _TRACE_SIMULATION_STEPS
    std::cout << "Record psp/conductances ..." << std::endl;
#endif
    for (unsigned int i=0; i < recorders.size(); i++) {
        if (recorders[i])
            recorders[i]->record_targets();
    }

    ////////////////////////////////
    // Update random distributions
    ////////////////////////////////
#ifdef _TRACE_SIMULATION_STEPS
    std::cout << "Draw required random numbers ..." << std::endl;
#endif

	pop0.update_rng();
	pop1.update_rng();
	pop2.update_rng();
	pop3.update_rng();
	pop4.update_rng();
	pop5.update_rng();
	pop6.update_rng();
	pop7.update_rng();
	pop9.update_rng();
	pop10.update_rng();
	pop11.update_rng();
	pop12.update_rng();
	pop13.update_rng();
	pop14.update_rng();
	pop15.update_rng();
	pop16.update_rng();
	pop17.update_rng();
	pop18.update_rng();
	pop19.update_rng();
	pop20.update_rng();
	pop22.update_rng();
	pop23.update_rng();
	pop24.update_rng();
	pop25.update_rng();
	pop26.update_rng();
	pop27.update_rng();
	pop28.update_rng();
	pop29.update_rng();



    ////////////////////////////////
    // Update neural variables
    ////////////////////////////////
#ifdef _TRACE_SIMULATION_STEPS
    std::cout << "Evaluate neural ODEs ..." << std::endl;
#endif

	pop0.update();
	pop1.update();
	pop2.update();
	pop3.update();
	pop4.update();
	pop5.update();
	pop6.update();
	pop7.update();
	pop8.update();
	pop9.update();
	pop10.update();
	pop11.update();
	pop12.update();
	pop13.update();
	pop14.update();
	pop15.update();
	pop16.update();
	pop17.update();
	pop18.update();
	pop19.update();
	pop20.update();
	pop21.update();
	pop22.update();
	pop23.update();
	pop24.update();
	pop25.update();
	pop26.update();
	pop27.update();
	pop28.update();
	pop29.update();



    ////////////////////////////////
    // Delay outputs
    ////////////////////////////////
#ifdef _TRACE_SIMULATION_STEPS
    std::cout << "Update delay queues ..." << std::endl;
#endif


    ////////////////////////////////
    // Global operations (min/max/mean)
    ////////////////////////////////
#ifdef _TRACE_SIMULATION_STEPS
    std::cout << "Update global operations ..." << std::endl;
#endif

	pop0.update_global_ops();
	pop1.update_global_ops();
	pop2.update_global_ops();
	pop3.update_global_ops();
	pop4.update_global_ops();
	pop5.update_global_ops();
	pop6.update_global_ops();
	pop7.update_global_ops();
	pop12.update_global_ops();
	pop16.update_global_ops();
	pop17.update_global_ops();
	pop19.update_global_ops();
	pop20.update_global_ops();



    ////////////////////////////////
    // Update synaptic variables
    ////////////////////////////////
#ifdef _TRACE_SIMULATION_STEPS
    std::cout << "Evaluate synaptic ODEs ..." << std::endl;
#endif

	proj0.update_synapse();
	proj2.update_synapse();
	proj9.update_synapse();
	proj11.update_synapse();
	proj13.update_synapse();
	proj15.update_synapse();
	proj17.update_synapse();
	proj19.update_synapse();
	proj36.update_synapse();
	proj38.update_synapse();
	proj49.update_synapse();
	proj51.update_synapse();
	proj61.update_synapse();



    ////////////////////////////////
    // Postsynaptic events
    ////////////////////////////////




    ////////////////////////////////
    // Structural plasticity
    ////////////////////////////////


    ////////////////////////////////
    // Recording neural / synaptic variables
    ////////////////////////////////

    for (unsigned int i=0; i < recorders.size(); i++){
        if (recorders[i])
            recorders[i]->record();
    }


    ////////////////////////////////
    // Increase internal time
    ////////////////////////////////
    t++;


}

// Simulate the network for the given number of steps,
// called from python
void run(const int nbSteps) {
#ifdef _TRACE_SIMULATION_STEPS
    std::cout << "Perform simulation for " << nbSteps << " steps." << std::endl;
#endif

    for(int i=0; i<nbSteps; i++) {
        singleStep();
    }

}

// Simulate the network for a single steps,
// called from python
void step() {

    singleStep();

}

int run_until(const int steps, std::vector<int> populations, bool or_and)
{


    run(steps);
    return steps;


}

/*
 *  Initialization methods
 */
// Initialize the internal data and the random numbers generator
void initialize(const double _dt) {


    // Internal variables
    dt = _dt;
    t = static_cast<long int>(0);

    // Populations
    // Initialize populations
    pop0.init_population();
    pop1.init_population();
    pop2.init_population();
    pop3.init_population();
    pop4.init_population();
    pop5.init_population();
    pop6.init_population();
    pop7.init_population();
    pop8.init_population();
    pop9.init_population();
    pop10.init_population();
    pop11.init_population();
    pop12.init_population();
    pop13.init_population();
    pop14.init_population();
    pop15.init_population();
    pop16.init_population();
    pop17.init_population();
    pop18.init_population();
    pop19.init_population();
    pop20.init_population();
    pop21.init_population();
    pop22.init_population();
    pop23.init_population();
    pop24.init_population();
    pop25.init_population();
    pop26.init_population();
    pop27.init_population();
    pop28.init_population();
    pop29.init_population();


    // Projections
    // Initialize projections
    proj0.init_projection();
    proj1.init_projection();
    proj2.init_projection();
    proj3.init_projection();
    proj4.init_projection();
    proj5.init_projection();
    proj6.init_projection();
    proj7.init_projection();
    proj8.init_projection();
    proj9.init_projection();
    proj10.init_projection();
    proj11.init_projection();
    proj12.init_projection();
    proj13.init_projection();
    proj14.init_projection();
    proj15.init_projection();
    proj16.init_projection();
    proj17.init_projection();
    proj18.init_projection();
    proj19.init_projection();
    proj20.init_projection();
    proj21.init_projection();
    proj22.init_projection();
    proj23.init_projection();
    proj24.init_projection();
    proj25.init_projection();
    proj26.init_projection();
    proj27.init_projection();
    proj28.init_projection();
    proj29.init_projection();
    proj30.init_projection();
    proj31.init_projection();
    proj32.init_projection();
    proj33.init_projection();
    proj34.init_projection();
    proj35.init_projection();
    proj36.init_projection();
    proj37.init_projection();
    proj38.init_projection();
    proj39.init_projection();
    proj40.init_projection();
    proj41.init_projection();
    proj42.init_projection();
    proj43.init_projection();
    proj44.init_projection();
    proj45.init_projection();
    proj46.init_projection();
    proj47.init_projection();
    proj48.init_projection();
    proj49.init_projection();
    proj50.init_projection();
    proj51.init_projection();
    proj52.init_projection();
    proj53.init_projection();
    proj54.init_projection();
    proj55.init_projection();
    proj56.init_projection();
    proj57.init_projection();
    proj58.init_projection();
    proj59.init_projection();
    proj60.init_projection();
    proj61.init_projection();
    proj62.init_projection();
    proj63.init_projection();
    proj64.init_projection();
    proj65.init_projection();
    proj66.init_projection();
    proj67.init_projection();
    proj68.init_projection();
    proj69.init_projection();
    proj70.init_projection();
    proj71.init_projection();
    proj72.init_projection();
    proj73.init_projection();
    proj74.init_projection();
    proj75.init_projection();
    proj76.init_projection();
    proj77.init_projection();
    proj78.init_projection();
    proj79.init_projection();
    proj80.init_projection();
    proj81.init_projection();
    proj82.init_projection();
    proj83.init_projection();
    proj84.init_projection();


    // Custom constants


}

// Change the seed of the RNG
void setSeed(const long int seed, const int num_sources, const bool use_seed_seq) {
    if (num_sources > 1)
        std::cerr << "WARNING - ANNarchy::setSeed(): num_sources should be 1 for single thread code." << std::endl;

    rng.clear();

    rng.push_back(std::mt19937(seed));

    rng.shrink_to_fit();
}

/*
 * Access to time and dt
 */
long int getTime() {return t;}
void setTime(const long int t_) { t=t_;}
double getDt() { return dt;}
void setDt(const double dt_) { dt=dt_;}

/*
 * Number of threads
 *
*/
void setNumberThreads(const int threads, const std::vector<int> core_list)
{
    if (threads > 1) {
        std::cerr << "WARNING: a call of setNumberThreads() is without effect on single thread simulation code." << std::endl;
    }

    if (core_list.size()>1) {
        std::cerr << "The provided core list is ambiguous and therefore ignored." << std::endl;
        return;
    }

#ifdef __linux__
    // set a cpu mask to prevent moving of threads
    cpu_set_t mask;

    // no CPUs selected
    CPU_ZERO(&mask);

    // no proc_bind
    for(auto it = core_list.begin(); it != core_list.end(); it++)
        CPU_SET(*it, &mask);
    const int set_result = sched_setaffinity(0, sizeof(cpu_set_t), &mask);
#else
    if (!core_list.empty()) {
        std::cout << "WARNING: manipulation of CPU masks is only available for linux systems." << std::endl;
    }
#endif
}
