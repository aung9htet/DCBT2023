/*
 *  ANNarchy-version: 4.7.2.6
 */
#pragma once

#include "ANNarchy.h"
#include "LILMatrix.hpp"




extern PopStruct2 pop2;
extern PopStruct8 pop8;
extern double dt;
extern long int t;

extern std::vector<std::mt19937> rng;

/////////////////////////////////////////////////////////////////////////////
// proj19: reward_prediction_error -> ventral_tegmental_area with target inh
/////////////////////////////////////////////////////////////////////////////
struct ProjStruct19 : LILMatrix<int, int> {
    ProjStruct19() : LILMatrix<int, int>( 1, 24) {
    }


    bool init_from_lil( std::vector<int> row_indices,
                        std::vector< std::vector<int> > column_indices,
                        std::vector< std::vector<double> > values,
                        std::vector< std::vector<int> > delays) {
        bool success = static_cast<LILMatrix<int, int>*>(this)->init_matrix_from_lil(row_indices, column_indices);
        if (!success)
            return false;


        // Local variable w
        w = init_matrix_variable<double>(static_cast<double>(0.0));
        update_matrix_variable_all<double>(w, values);


        // init other variables than 'w' or delay
        if (!init_attributes()){
            return false;
        }

    #ifdef _DEBUG_CONN
        static_cast<LILMatrix<int, int>*>(this)->print_data_representation();
    #endif
        return true;
    }





    // Transmission and plasticity flags
    bool _transmission, _plasticity, _update;
    int _update_period;
    long int _update_offset;





    // Local parameter tau
    std::vector< std::vector<double > > tau;

    // Local parameter baseline_dopa
    std::vector< std::vector<double > > baseline_dopa;

    // Local variable aux
    std::vector< std::vector<double > > aux;

    // Local variable delta
    std::vector< std::vector<double > > delta;

    // Local variable w
    std::vector< std::vector<double > > w;




    // Method called to allocate/initialize the variables
    bool init_attributes() {

        // Local parameter tau
        tau = init_matrix_variable<double>(static_cast<double>(0.0));

        // Local parameter baseline_dopa
        baseline_dopa = init_matrix_variable<double>(static_cast<double>(0.0));

        // Local variable aux
        aux = init_matrix_variable<double>(static_cast<double>(0.0));

        // Local variable delta
        delta = init_matrix_variable<double>(static_cast<double>(0.0));




        return true;
    }

    // Method called to initialize the projection
    void init_projection() {
    #ifdef _DEBUG
        std::cout << "ProjStruct19::init_projection() - this = " << this << std::endl;
    #endif

        _transmission = true;
        _update = true;
        _plasticity = true;
        _update_period = 1;
        _update_offset = 0L;

        init_attributes();



    }

    // Spiking networks: reset the ring buffer when non-uniform
    void reset_ring_buffer() {

    }

    // Spiking networks: update maximum delay when non-uniform
    void update_max_delay(int d){

    }

    // Computes the weighted sum of inputs or updates the conductances
    void compute_psp() {
    #ifdef _TRACE_SIMULATION_STEPS
        std::cout << "    ProjStruct19::compute_psp()" << std::endl;
    #endif


    #ifdef __AVX__
        if (_transmission && pop8._active) {
            int _s, _stop;
            double _tmp_sum[4];
            double* __restrict__ _pre_r = pop2.r.data();

            for (int i = 0; i < post_rank.size(); i++) {
                int rk_post = post_rank[i];
                int* __restrict__ _idx = pre_rank[i].data();
                double* __restrict__ _w = w[i].data();

                _s = 0;
                _stop = pre_rank[i].size();
                __m256d _tmp_reg_sum = _mm256_setzero_pd();

                for (; (_s+8) < _stop; _s+=8) {
                    __m256d _tmp_r = _mm256_set_pd(
                        _pre_r[_idx[_s+3]], _pre_r[_idx[_s+2]], _pre_r[_idx[_s+1]], _pre_r[_idx[_s]]
                    );
                    __m256d _tmp_r2 = _mm256_set_pd(
                        _pre_r[_idx[_s+7]], _pre_r[_idx[_s+6]], _pre_r[_idx[_s+5]], _pre_r[_idx[_s+4]]
                    );

                    __m256d _tmp_w = _mm256_loadu_pd(&_w[_s]);
                    __m256d _tmp_w2 = _mm256_loadu_pd(&_w[_s+4]);

                    _tmp_reg_sum = _mm256_add_pd(_tmp_reg_sum, _mm256_mul_pd(_tmp_r, _tmp_w));
                    _tmp_reg_sum = _mm256_add_pd(_tmp_reg_sum, _mm256_mul_pd(_tmp_r2, _tmp_w2));
                }

                _mm256_storeu_pd(_tmp_sum, _tmp_reg_sum);

                // partial sums
                double lsum = _tmp_sum[0] + _tmp_sum[1] + _tmp_sum[2] + _tmp_sum[3];

                // remainder loop
                for (; _s < _stop; _s++)
                    lsum += _pre_r[_idx[_s]] * _w[_s];

                pop8._sum_inh[rk_post] += lsum;
            }
        } // active
    #else
        std::cerr << "The code was not compiled with AVX support. Please check your compiler flags ..." << std::endl;
    #endif

    }

    // Draws random numbers
    void update_rng() {

    }

    // Updates synaptic variables
    void update_synapse() {
    #ifdef _TRACE_SIMULATION_STEPS
        std::cout << "    ProjStruct19::update_synapse()" << std::endl;
    #endif

        int rk_post, rk_pre;
        double _dt = dt * _update_period;

        // Check periodicity
        if(_transmission && _update && pop8._active && ( (t - _update_offset)%_update_period == 0L) ){
            // Global variables


            // Semiglobal/Local variables
            for (int i = 0; i < post_rank.size(); i++) {
                rk_post = post_rank[i]; // Get postsynaptic rank

                // Semi-global variables


                // Local variables
                for (int j = 0; j < pre_rank[i].size(); j++) {
                    rk_pre = pre_rank[i][j]; // Get presynaptic rank

                    // aux = if (post.sum(exc) > 0.001): 1.0 else: 2.5
                    aux[i][j] = (pop8._sum_exc[rk_post] > 0.001 ? 1.0 : 2.5);


                    // delta = aux * (post.r - baseline_dopa) * pos(pre.r - mean(pre.r))
                    delta[i][j] = aux[i][j]*(pop8.r[rk_post] - baseline_dopa[i][j])*positive(-pop2._mean_r + pop2.r[rk_pre]);


                    // tau * dw/_dt = delta
                    double _w = delta[i][j]/tau[i][j];

                    // tau * dw/_dt = delta
                    if(_plasticity){
                    w[i][j] += _dt*_w ;
                    if(w[i][j] < 0.0)
                        w[i][j] = 0.0;

                    }

                }
            }
        }

    }

    // Post-synaptic events
    void post_event() {

    }

    // Variable/Parameter access methods

    std::vector<std::vector<double>> get_local_attribute_all_double(std::string name) {
    #ifdef _DEBUG
        std::cout << "ProjStruct19::get_local_attribute_all_double(name = "<<name<<")" << std::endl;
    #endif

        // Local parameter tau
        if ( name.compare("tau") == 0 ) {

            return get_matrix_variable_all<double>(tau);
        }

        // Local parameter baseline_dopa
        if ( name.compare("baseline_dopa") == 0 ) {

            return get_matrix_variable_all<double>(baseline_dopa);
        }

        // Local variable aux
        if ( name.compare("aux") == 0 ) {

            return get_matrix_variable_all<double>(aux);
        }

        // Local variable delta
        if ( name.compare("delta") == 0 ) {

            return get_matrix_variable_all<double>(delta);
        }

        // Local variable w
        if ( name.compare("w") == 0 ) {

            return get_matrix_variable_all<double>(w);
        }


        // should not happen
        std::cerr << "ProjStruct19::get_local_attribute_all_double: " << name << " not found" << std::endl;
        return std::vector<std::vector<double>>();
    }

    std::vector<double> get_local_attribute_row_double(std::string name, int rk_post) {
    #ifdef _DEBUG
        std::cout << "ProjStruct19::get_local_attribute_row_double(name = "<<name<<", rk_post = "<<rk_post<<")" << std::endl;
    #endif

        // Local parameter tau
        if ( name.compare("tau") == 0 ) {

            return get_matrix_variable_row<double>(tau, rk_post);
        }

        // Local parameter baseline_dopa
        if ( name.compare("baseline_dopa") == 0 ) {

            return get_matrix_variable_row<double>(baseline_dopa, rk_post);
        }

        // Local variable aux
        if ( name.compare("aux") == 0 ) {

            return get_matrix_variable_row<double>(aux, rk_post);
        }

        // Local variable delta
        if ( name.compare("delta") == 0 ) {

            return get_matrix_variable_row<double>(delta, rk_post);
        }

        // Local variable w
        if ( name.compare("w") == 0 ) {

            return get_matrix_variable_row<double>(w, rk_post);
        }


        // should not happen
        std::cerr << "ProjStruct19::get_local_attribute_row_double: " << name << " not found" << std::endl;
        return std::vector<double>();
    }

    double get_local_attribute_double(std::string name, int rk_post, int rk_pre) {
    #ifdef _DEBUG
        std::cout << "ProjStruct19::get_local_attribute_row_double(name = "<<name<<", rk_post = "<<rk_post<<", rk_pre = "<<rk_pre<<")" << std::endl;
    #endif

        // Local parameter tau
        if ( name.compare("tau") == 0 ) {

            return get_matrix_variable<double>(tau, rk_post, rk_pre);
        }

        // Local parameter baseline_dopa
        if ( name.compare("baseline_dopa") == 0 ) {

            return get_matrix_variable<double>(baseline_dopa, rk_post, rk_pre);
        }

        // Local variable aux
        if ( name.compare("aux") == 0 ) {

            return get_matrix_variable<double>(aux, rk_post, rk_pre);
        }

        // Local variable delta
        if ( name.compare("delta") == 0 ) {

            return get_matrix_variable<double>(delta, rk_post, rk_pre);
        }

        // Local variable w
        if ( name.compare("w") == 0 ) {

            return get_matrix_variable<double>(w, rk_post, rk_pre);
        }


        // should not happen
        std::cerr << "ProjStruct19::get_local_attribute: " << name << " not found" << std::endl;
        return 0.0;
    }

    void set_local_attribute_all_double(std::string name, std::vector<std::vector<double>> value) {

        // Local parameter tau
        if ( name.compare("tau") == 0 ) {
            update_matrix_variable_all<double>(tau, value);

            return;
        }

        // Local parameter baseline_dopa
        if ( name.compare("baseline_dopa") == 0 ) {
            update_matrix_variable_all<double>(baseline_dopa, value);

            return;
        }

        // Local variable aux
        if ( name.compare("aux") == 0 ) {
            update_matrix_variable_all<double>(aux, value);

            return;
        }

        // Local variable delta
        if ( name.compare("delta") == 0 ) {
            update_matrix_variable_all<double>(delta, value);

            return;
        }

        // Local variable w
        if ( name.compare("w") == 0 ) {
            update_matrix_variable_all<double>(w, value);

            return;
        }

    }

    void set_local_attribute_row_double(std::string name, int rk_post, std::vector<double> value) {

        // Local parameter tau
        if ( name.compare("tau") == 0 ) {
            update_matrix_variable_row<double>(tau, rk_post, value);

            return;
        }

        // Local parameter baseline_dopa
        if ( name.compare("baseline_dopa") == 0 ) {
            update_matrix_variable_row<double>(baseline_dopa, rk_post, value);

            return;
        }

        // Local variable aux
        if ( name.compare("aux") == 0 ) {
            update_matrix_variable_row<double>(aux, rk_post, value);

            return;
        }

        // Local variable delta
        if ( name.compare("delta") == 0 ) {
            update_matrix_variable_row<double>(delta, rk_post, value);

            return;
        }

        // Local variable w
        if ( name.compare("w") == 0 ) {
            update_matrix_variable_row<double>(w, rk_post, value);

            return;
        }

    }

    void set_local_attribute_double(std::string name, int rk_post, int rk_pre, double value) {

        // Local parameter tau
        if ( name.compare("tau") == 0 ) {
            update_matrix_variable<double>(tau, rk_post, rk_pre, value);

            return;
        }

        // Local parameter baseline_dopa
        if ( name.compare("baseline_dopa") == 0 ) {
            update_matrix_variable<double>(baseline_dopa, rk_post, rk_pre, value);

            return;
        }

        // Local variable aux
        if ( name.compare("aux") == 0 ) {
            update_matrix_variable<double>(aux, rk_post, rk_pre, value);

            return;
        }

        // Local variable delta
        if ( name.compare("delta") == 0 ) {
            update_matrix_variable<double>(delta, rk_post, rk_pre, value);

            return;
        }

        // Local variable w
        if ( name.compare("w") == 0 ) {
            update_matrix_variable<double>(w, rk_post, rk_pre, value);

            return;
        }

    }


    // Access additional


    // Memory management
    long int size_in_bytes() {
        long int size_in_bytes = 0;

        // connectivity
        size_in_bytes += static_cast<LILMatrix<int, int>*>(this)->size_in_bytes();

        // Local variable aux
        size_in_bytes += sizeof(std::vector<std::vector<double>>);
        size_in_bytes += sizeof(std::vector<double>) * aux.capacity();
        for(auto it = aux.cbegin(); it != aux.cend(); it++)
            size_in_bytes += (it->capacity()) * sizeof(double);

        // Local variable delta
        size_in_bytes += sizeof(std::vector<std::vector<double>>);
        size_in_bytes += sizeof(std::vector<double>) * delta.capacity();
        for(auto it = delta.cbegin(); it != delta.cend(); it++)
            size_in_bytes += (it->capacity()) * sizeof(double);

        // Local variable w
        size_in_bytes += sizeof(std::vector<std::vector<double>>);
        size_in_bytes += sizeof(std::vector<double>) * w.capacity();
        for(auto it = w.cbegin(); it != w.cend(); it++)
            size_in_bytes += (it->capacity()) * sizeof(double);

        // Local parameter tau
        size_in_bytes += sizeof(std::vector<std::vector<double>>);
        size_in_bytes += sizeof(std::vector<double>) * tau.capacity();
        for(auto it = tau.cbegin(); it != tau.cend(); it++)
            size_in_bytes += (it->capacity()) * sizeof(double);

        // Local parameter baseline_dopa
        size_in_bytes += sizeof(std::vector<std::vector<double>>);
        size_in_bytes += sizeof(std::vector<double>) * baseline_dopa.capacity();
        for(auto it = baseline_dopa.cbegin(); it != baseline_dopa.cend(); it++)
            size_in_bytes += (it->capacity()) * sizeof(double);

        return size_in_bytes;
    }

    // Structural plasticity



    void clear() {
    #ifdef _DEBUG
        std::cout << "ProjStruct19::clear() - this = " << this << std::endl;
    #endif

        // Connectivity
        static_cast<LILMatrix<int, int>*>(this)->clear();

        // aux
        for (auto it = aux.begin(); it != aux.end(); it++) {
            it->clear();
            it->shrink_to_fit();
        };
        aux.clear();
        aux.shrink_to_fit();

        // delta
        for (auto it = delta.begin(); it != delta.end(); it++) {
            it->clear();
            it->shrink_to_fit();
        };
        delta.clear();
        delta.shrink_to_fit();

        // w
        for (auto it = w.begin(); it != w.end(); it++) {
            it->clear();
            it->shrink_to_fit();
        };
        w.clear();
        w.shrink_to_fit();

        // tau
        for (auto it = tau.begin(); it != tau.end(); it++) {
            it->clear();
            it->shrink_to_fit();
        };
        tau.clear();
        tau.shrink_to_fit();

        // baseline_dopa
        for (auto it = baseline_dopa.begin(); it != baseline_dopa.end(); it++) {
            it->clear();
            it->shrink_to_fit();
        };
        baseline_dopa.clear();
        baseline_dopa.shrink_to_fit();

    }
};

