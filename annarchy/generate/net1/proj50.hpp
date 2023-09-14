/*
 *  ANNarchy-version: 4.7.2.6
 */
#pragma once

#include "ANNarchy.h"
#include "LILMatrix.hpp"




extern PopStruct17 pop17;
extern PopStruct17 pop17;
extern double dt;
extern long int t;

extern std::vector<std::mt19937> rng;

/////////////////////////////////////////////////////////////////////////////
// proj50: striatum_d2_dorsomedial -> striatum_d2_dorsomedial with target inh
/////////////////////////////////////////////////////////////////////////////
struct ProjStruct50 : LILMatrix<int, int> {
    ProjStruct50() : LILMatrix<int, int>( 24, 24) {
    }


    bool init_from_lil( std::vector<int> row_indices,
                        std::vector< std::vector<int> > column_indices,
                        std::vector< std::vector<double> > values,
                        std::vector< std::vector<int> > delays) {
        bool success = static_cast<LILMatrix<int, int>*>(this)->init_matrix_from_lil(row_indices, column_indices);
        if (!success)
            return false;

        w = values[0][0];


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





    // Global parameter w
    double  w ;




    // Method called to allocate/initialize the variables
    bool init_attributes() {




        return true;
    }

    // Method called to initialize the projection
    void init_projection() {
    #ifdef _DEBUG
        std::cout << "ProjStruct50::init_projection() - this = " << this << std::endl;
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
        std::cout << "    ProjStruct50::compute_psp()" << std::endl;
    #endif


    #ifdef __AVX__
        if (_transmission && pop17._active) {
            int _s, _stop;
            double _tmp_sum[4];
            double* __restrict__ _pre_r = pop17.r.data();

            for (int i = 0; i < post_rank.size(); i++) {
                int rk_post = post_rank[i];
                int* __restrict__ _idx = pre_rank[i].data();
                _stop = pre_rank[i].size();

                __m256d _tmp_reg_sum = _mm256_setzero_pd();
                _s = 0;
                for (; (_s+8) < _stop; _s+=8) {
                    __m256d _tmp_r = _mm256_set_pd(
                        _pre_r[_idx[_s+3]], _pre_r[_idx[_s+2]], _pre_r[_idx[_s+1]], _pre_r[_idx[_s]]
                    );
                    __m256d _tmp_r2 = _mm256_set_pd(
                        _pre_r[_idx[_s+7]], _pre_r[_idx[_s+6]], _pre_r[_idx[_s+5]], _pre_r[_idx[_s+4]]
                    );

                    _tmp_reg_sum = _mm256_add_pd(_tmp_reg_sum, _tmp_r);
                    _tmp_reg_sum = _mm256_add_pd(_tmp_reg_sum, _tmp_r2);
                }
                _mm256_storeu_pd(_tmp_sum, _tmp_reg_sum);

                // partial sums
                double lsum = _tmp_sum[0] + _tmp_sum[1] + _tmp_sum[2] + _tmp_sum[3];

                // remainder loop
                for (; _s < _stop; _s++)
                    lsum += _pre_r[_idx[_s]];

                pop17._sum_inh[rk_post] += w * lsum;
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
        std::cout << "    ProjStruct50::update_synapse()" << std::endl;
    #endif


    }

    // Post-synaptic events
    void post_event() {

    }

    // Variable/Parameter access methods

    double get_global_attribute_double(std::string name) {

        // Global parameter w
        if ( name.compare("w") == 0 ) {

            return w;
        }


        // should not happen
        std::cerr << "ProjStruct50::get_global_attribute_double: " << name << " not found" << std::endl;
        return 0.0;
    }

    void set_global_attribute_double(std::string name, double value) {

        // Global parameter w
        if ( name.compare("w") == 0 ) {
            w = value;

            return;
        }

    }


    // Access additional


    // Memory management
    long int size_in_bytes() {
        long int size_in_bytes = 0;

        // connectivity
        size_in_bytes += static_cast<LILMatrix<int, int>*>(this)->size_in_bytes();

        // Global parameter w
        size_in_bytes += sizeof(double);

        return size_in_bytes;
    }

    // Structural plasticity



    void clear() {
    #ifdef _DEBUG
        std::cout << "ProjStruct50::clear() - this = " << this << std::endl;
    #endif

        // Connectivity
        static_cast<LILMatrix<int, int>*>(this)->clear();

    }
};

