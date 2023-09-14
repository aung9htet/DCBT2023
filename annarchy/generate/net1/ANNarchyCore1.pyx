# cython: embedsignature=True
from cpython.exc cimport PyErr_CheckSignals
from libcpp.vector cimport vector
from libcpp.map cimport map, pair
from libcpp cimport bool
from libcpp.string cimport string
from math import ceil
import numpy as np
import sys
cimport numpy as np
cimport cython

# Short names for unsigned integer types
ctypedef unsigned char _ann_uint8
ctypedef unsigned short _ann_uint16
ctypedef unsigned int _ann_uint32
ctypedef unsigned long _ann_uint64

import ANNarchy
from ANNarchy.core.cython_ext.Connector cimport LILConnectivity as LIL

cdef extern from "ANNarchy.h":

    # User-defined functions


    # User-defined constants


    # Data structures

    # Export Population 0 (hippocampus)
    cdef struct PopStruct0 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 1 (hippocampus_activity)
    cdef struct PopStruct1 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 2 (reward_prediction_error)
    cdef struct PopStruct2 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 3 (nacc_shell_d1)
    cdef struct PopStruct3 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 4 (nacc_shell_d2)
    cdef struct PopStruct4 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 5 (stn_ventral)
    cdef struct PopStruct5 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 6 (vp_direct)
    cdef struct PopStruct6 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 7 (vp_indirect)
    cdef struct PopStruct7 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 8 (ventral_tegmental_area)
    cdef struct PopStruct8 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 9 (pptn_ventral)
    cdef struct PopStruct9 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 10 (cortical_feedback_ventral)
    cdef struct PopStruct10 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 11 (thalamus_ventral)
    cdef struct PopStruct11 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 12 (medial_pfc)
    cdef struct PopStruct12 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 13 (nacc_core)
    cdef struct PopStruct13 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 14 (substantia_nigra_pars_reticulata)
    cdef struct PopStruct14 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 15 (visual_input)
    cdef struct PopStruct15 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 16 (striatum_d1_dorsomedial)
    cdef struct PopStruct16 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 17 (striatum_d2_dorsomedial)
    cdef struct PopStruct17 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 18 (stn_dorsomedial)
    cdef struct PopStruct18 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 19 (gpi_dorsomedial)
    cdef struct PopStruct19 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 20 (gpe_dorsomedial)
    cdef struct PopStruct20 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 21 (substantia_nigra_pars_compacta)
    cdef struct PopStruct21 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 22 (pptn_dorsomedial)
    cdef struct PopStruct22 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 23 (cortical_feedback_dorsomedial)
    cdef struct PopStruct23 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 24 (thalamus_dorsomedial)
    cdef struct PopStruct24 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 25 (dorsolateral_pfc)
    cdef struct PopStruct25 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 26 (striatum_d1_dorsolateral)
    cdef struct PopStruct26 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 27 (gpi_dorsolateral)
    cdef struct PopStruct27 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 28 (thalamus_dorsolateral)
    cdef struct PopStruct28 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()

    # Export Population 29 (premotor)
    cdef struct PopStruct29 :
        # Number of neurons
        int get_size()
        void set_size(int)
        # Maximum delay in steps
        int get_max_delay()
        void set_max_delay(int)
        void update_max_delay(int)
        # Activate/deactivate the population
        bool is_active()
        void set_active(bool)
        # Reset the population
        void reset()


        # Local attributes
        vector[double] get_local_attribute_all_double(string)
        double get_local_attribute_double(string, int)
        void set_local_attribute_all_double(string, vector[double])
        void set_local_attribute_double(string, int, double)





        # memory management
        long int size_in_bytes()
        void clear()


    # Export Projection 0
    cdef struct ProjStruct0 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 1
    cdef struct ProjStruct1 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 2
    cdef struct ProjStruct2 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 3
    cdef struct ProjStruct3 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)

        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 4
    cdef struct ProjStruct4 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)

        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 5
    cdef struct ProjStruct5 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)

        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 6
    cdef struct ProjStruct6 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)

        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 7
    cdef struct ProjStruct7 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 8
    cdef struct ProjStruct8 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 9
    cdef struct ProjStruct9 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 10
    cdef struct ProjStruct10 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 11
    cdef struct ProjStruct11 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 12
    cdef struct ProjStruct12 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 13
    cdef struct ProjStruct13 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 14
    cdef struct ProjStruct14 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 15
    cdef struct ProjStruct15 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 16
    cdef struct ProjStruct16 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 17
    cdef struct ProjStruct17 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 18
    cdef struct ProjStruct18 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 19
    cdef struct ProjStruct19 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 20
    cdef struct ProjStruct20 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 21
    cdef struct ProjStruct21 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 22
    cdef struct ProjStruct22 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 23
    cdef struct ProjStruct23 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 24
    cdef struct ProjStruct24 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 25
    cdef struct ProjStruct25 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 26
    cdef struct ProjStruct26 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 27
    cdef struct ProjStruct27 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 28
    cdef struct ProjStruct28 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 29
    cdef struct ProjStruct29 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 30
    cdef struct ProjStruct30 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 31
    cdef struct ProjStruct31 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 32
    cdef struct ProjStruct32 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 33
    cdef struct ProjStruct33 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 34
    cdef struct ProjStruct34 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 35
    cdef struct ProjStruct35 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 36
    cdef struct ProjStruct36 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 37
    cdef struct ProjStruct37 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 38
    cdef struct ProjStruct38 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 39
    cdef struct ProjStruct39 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)

        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 40
    cdef struct ProjStruct40 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)

        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 41
    cdef struct ProjStruct41 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)

        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 42
    cdef struct ProjStruct42 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)

        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 43
    cdef struct ProjStruct43 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 44
    cdef struct ProjStruct44 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 45
    cdef struct ProjStruct45 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 46
    cdef struct ProjStruct46 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 47
    cdef struct ProjStruct47 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 48
    cdef struct ProjStruct48 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 49
    cdef struct ProjStruct49 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 50
    cdef struct ProjStruct50 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 51
    cdef struct ProjStruct51 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 52
    cdef struct ProjStruct52 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 53
    cdef struct ProjStruct53 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 54
    cdef struct ProjStruct54 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 55
    cdef struct ProjStruct55 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 56
    cdef struct ProjStruct56 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 57
    cdef struct ProjStruct57 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 58
    cdef struct ProjStruct58 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 59
    cdef struct ProjStruct59 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 60
    cdef struct ProjStruct60 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 61
    cdef struct ProjStruct61 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Local Attributes
        vector[vector[double]] get_local_attribute_all_double(string)
        vector[double] get_local_attribute_row_double(string, int)
        double get_local_attribute_double(string, int, int)
        void set_local_attribute_all_double(string, vector[vector[double]])
        void set_local_attribute_row_double(string, int, vector[double])
        void set_local_attribute_double(string, int, int, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 62
    cdef struct ProjStruct62 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 63
    cdef struct ProjStruct63 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 64
    cdef struct ProjStruct64 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 65
    cdef struct ProjStruct65 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 66
    cdef struct ProjStruct66 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 67
    cdef struct ProjStruct67 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 68
    cdef struct ProjStruct68 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 69
    cdef struct ProjStruct69 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 70
    cdef struct ProjStruct70 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 71
    cdef struct ProjStruct71 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 72
    cdef struct ProjStruct72 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 73
    cdef struct ProjStruct73 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 74
    cdef struct ProjStruct74 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 75
    cdef struct ProjStruct75 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 76
    cdef struct ProjStruct76 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 77
    cdef struct ProjStruct77 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 78
    cdef struct ProjStruct78 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 79
    cdef struct ProjStruct79 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 80
    cdef struct ProjStruct80 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 81
    cdef struct ProjStruct81 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 82
    cdef struct ProjStruct82 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 83
    cdef struct ProjStruct83 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()

    # Export Projection 84
    cdef struct ProjStruct84 :
        # Flags
        bool _transmission
        bool _plasticity
        bool _update
        int _update_period
        long _update_offset

        # Connectivity
        bool init_from_lil(vector[int], vector[vector[int]], vector[vector[double]], vector[vector[int]])
        # Access connectivity
        vector[int] get_post_rank()
        vector[ vector[int] ] get_pre_ranks()
        vector[int] get_dendrite_pre_rank(int)
        int nb_synapses()
        int nb_dendrites()
        int dendrite_size(int)





        # Global Attributes
        double get_global_attribute_double(string)
        void set_global_attribute_double(string, double)





        # cuda configuration


        # memory management
        long int size_in_bytes()
        void clear()



    # Monitors
    cdef cppclass Monitor:
        vector[int] ranks
        int period_
        int period_offset_
        long offset_


    # Population 0 (hippocampus) : Monitor
    cdef cppclass PopRecorder0 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder0* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
    # Population 1 (hippocampus_activity) : Monitor
    cdef cppclass PopRecorder1 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder1* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 2 (reward_prediction_error) : Monitor
    cdef cppclass PopRecorder2 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder2* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 3 (nacc_shell_d1) : Monitor
    cdef cppclass PopRecorder3 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder3* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        vector[vector[double]] trace
        bool record_trace

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 4 (nacc_shell_d2) : Monitor
    cdef cppclass PopRecorder4 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder4* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        vector[vector[double]] trace
        bool record_trace

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 5 (stn_ventral) : Monitor
    cdef cppclass PopRecorder5 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder5* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        vector[vector[double]] trace
        bool record_trace

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 6 (vp_direct) : Monitor
    cdef cppclass PopRecorder6 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder6* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        vector[vector[double]] trace
        bool record_trace

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 7 (vp_indirect) : Monitor
    cdef cppclass PopRecorder7 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder7* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        vector[vector[double]] trace
        bool record_trace

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 8 (ventral_tegmental_area) : Monitor
    cdef cppclass PopRecorder8 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder8* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] ex_in
        bool record_ex_in

        vector[vector[double]] s_inh
        bool record_s_inh

        vector[vector[double]] aux
        bool record_aux

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 9 (pptn_ventral) : Monitor
    cdef cppclass PopRecorder9 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder9* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
    # Population 10 (cortical_feedback_ventral) : Monitor
    cdef cppclass PopRecorder10 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder10* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] cortex_input
        bool record_cortex_input

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 11 (thalamus_ventral) : Monitor
    cdef cppclass PopRecorder11 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder11* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 12 (medial_pfc) : Monitor
    cdef cppclass PopRecorder12 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder12* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 13 (nacc_core) : Monitor
    cdef cppclass PopRecorder13 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder13* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 14 (substantia_nigra_pars_reticulata) : Monitor
    cdef cppclass PopRecorder14 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder14* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 15 (visual_input) : Monitor
    cdef cppclass PopRecorder15 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder15* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 16 (striatum_d1_dorsomedial) : Monitor
    cdef cppclass PopRecorder16 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder16* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        vector[vector[double]] trace
        bool record_trace

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 17 (striatum_d2_dorsomedial) : Monitor
    cdef cppclass PopRecorder17 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder17* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        vector[vector[double]] trace
        bool record_trace

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 18 (stn_dorsomedial) : Monitor
    cdef cppclass PopRecorder18 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder18* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 19 (gpi_dorsomedial) : Monitor
    cdef cppclass PopRecorder19 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder19* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 20 (gpe_dorsomedial) : Monitor
    cdef cppclass PopRecorder20 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder20* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_dopa
        bool record__sum_dopa

        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 21 (substantia_nigra_pars_compacta) : Monitor
    cdef cppclass PopRecorder21 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder21* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] ex_in
        bool record_ex_in

        vector[vector[double]] s_inh
        bool record_s_inh

        vector[vector[double]] aux
        bool record_aux

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 22 (pptn_dorsomedial) : Monitor
    cdef cppclass PopRecorder22 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder22* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
    # Population 23 (cortical_feedback_dorsomedial) : Monitor
    cdef cppclass PopRecorder23 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder23* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] cortex_input
        bool record_cortex_input

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 24 (thalamus_dorsomedial) : Monitor
    cdef cppclass PopRecorder24 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder24* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 25 (dorsolateral_pfc) : Monitor
    cdef cppclass PopRecorder25 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder25* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 26 (striatum_d1_dorsolateral) : Monitor
    cdef cppclass PopRecorder26 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder26* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 27 (gpi_dorsolateral) : Monitor
    cdef cppclass PopRecorder27 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder27* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 28 (thalamus_dorsolateral) : Monitor
    cdef cppclass PopRecorder28 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder28* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Population 29 (premotor) : Monitor
    cdef cppclass PopRecorder29 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        PopRecorder29* get_instance(int)
        long int size_in_bytes()
        void clear()

        vector[vector[double]] mp
        bool record_mp

        vector[vector[double]] r
        bool record_r

        # Targets
        vector[vector[double]] _sum_exc
        bool record__sum_exc

        vector[vector[double]] _sum_inh
        bool record__sum_inh

    # Projection 0 : Monitor
    cdef cppclass ProjRecorder0 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder0* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] condition_0
        bool record_condition_0

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 1 : Monitor
    cdef cppclass ProjRecorder1 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder1* get_instance(int)

    # Projection 2 : Monitor
    cdef cppclass ProjRecorder2 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder2* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] aux
        bool record_aux

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 3 : Monitor
    cdef cppclass ProjRecorder3 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder3* get_instance(int)

    # Projection 4 : Monitor
    cdef cppclass ProjRecorder4 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder4* get_instance(int)

    # Projection 5 : Monitor
    cdef cppclass ProjRecorder5 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder5* get_instance(int)

    # Projection 6 : Monitor
    cdef cppclass ProjRecorder6 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder6* get_instance(int)

    # Projection 7 : Monitor
    cdef cppclass ProjRecorder7 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder7* get_instance(int)

    # Projection 8 : Monitor
    cdef cppclass ProjRecorder8 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder8* get_instance(int)

    # Projection 9 : Monitor
    cdef cppclass ProjRecorder9 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder9* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] condition_0
        bool record_condition_0

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 10 : Monitor
    cdef cppclass ProjRecorder10 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder10* get_instance(int)

    # Projection 11 : Monitor
    cdef cppclass ProjRecorder11 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder11* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] aux
        bool record_aux

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 12 : Monitor
    cdef cppclass ProjRecorder12 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder12* get_instance(int)

    # Projection 13 : Monitor
    cdef cppclass ProjRecorder13 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder13* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] condition_0
        bool record_condition_0

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 14 : Monitor
    cdef cppclass ProjRecorder14 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder14* get_instance(int)

    # Projection 15 : Monitor
    cdef cppclass ProjRecorder15 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder15* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] aux
        bool record_aux

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 16 : Monitor
    cdef cppclass ProjRecorder16 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder16* get_instance(int)

    # Projection 17 : Monitor
    cdef cppclass ProjRecorder17 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder17* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] condition_0
        bool record_condition_0

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 18 : Monitor
    cdef cppclass ProjRecorder18 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder18* get_instance(int)

    # Projection 19 : Monitor
    cdef cppclass ProjRecorder19 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder19* get_instance(int)

        vector[vector[vector[double]]] aux
        bool record_aux

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 20 : Monitor
    cdef cppclass ProjRecorder20 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder20* get_instance(int)

    # Projection 21 : Monitor
    cdef cppclass ProjRecorder21 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder21* get_instance(int)

    # Projection 22 : Monitor
    cdef cppclass ProjRecorder22 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder22* get_instance(int)

    # Projection 23 : Monitor
    cdef cppclass ProjRecorder23 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder23* get_instance(int)

    # Projection 24 : Monitor
    cdef cppclass ProjRecorder24 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder24* get_instance(int)

    # Projection 25 : Monitor
    cdef cppclass ProjRecorder25 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder25* get_instance(int)

    # Projection 26 : Monitor
    cdef cppclass ProjRecorder26 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder26* get_instance(int)

    # Projection 27 : Monitor
    cdef cppclass ProjRecorder27 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder27* get_instance(int)

    # Projection 28 : Monitor
    cdef cppclass ProjRecorder28 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder28* get_instance(int)

    # Projection 29 : Monitor
    cdef cppclass ProjRecorder29 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder29* get_instance(int)

    # Projection 30 : Monitor
    cdef cppclass ProjRecorder30 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder30* get_instance(int)

    # Projection 31 : Monitor
    cdef cppclass ProjRecorder31 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder31* get_instance(int)

    # Projection 32 : Monitor
    cdef cppclass ProjRecorder32 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder32* get_instance(int)

    # Projection 33 : Monitor
    cdef cppclass ProjRecorder33 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder33* get_instance(int)

    # Projection 34 : Monitor
    cdef cppclass ProjRecorder34 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder34* get_instance(int)

    # Projection 35 : Monitor
    cdef cppclass ProjRecorder35 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder35* get_instance(int)

    # Projection 36 : Monitor
    cdef cppclass ProjRecorder36 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder36* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] condition_0
        bool record_condition_0

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 37 : Monitor
    cdef cppclass ProjRecorder37 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder37* get_instance(int)

    # Projection 38 : Monitor
    cdef cppclass ProjRecorder38 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder38* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] aux
        bool record_aux

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 39 : Monitor
    cdef cppclass ProjRecorder39 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder39* get_instance(int)

    # Projection 40 : Monitor
    cdef cppclass ProjRecorder40 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder40* get_instance(int)

    # Projection 41 : Monitor
    cdef cppclass ProjRecorder41 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder41* get_instance(int)

    # Projection 42 : Monitor
    cdef cppclass ProjRecorder42 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder42* get_instance(int)

    # Projection 43 : Monitor
    cdef cppclass ProjRecorder43 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder43* get_instance(int)

    # Projection 44 : Monitor
    cdef cppclass ProjRecorder44 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder44* get_instance(int)

    # Projection 45 : Monitor
    cdef cppclass ProjRecorder45 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder45* get_instance(int)

    # Projection 46 : Monitor
    cdef cppclass ProjRecorder46 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder46* get_instance(int)

    # Projection 47 : Monitor
    cdef cppclass ProjRecorder47 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder47* get_instance(int)

    # Projection 48 : Monitor
    cdef cppclass ProjRecorder48 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder48* get_instance(int)

    # Projection 49 : Monitor
    cdef cppclass ProjRecorder49 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder49* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] condition_0
        bool record_condition_0

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 50 : Monitor
    cdef cppclass ProjRecorder50 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder50* get_instance(int)

    # Projection 51 : Monitor
    cdef cppclass ProjRecorder51 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder51* get_instance(int)

        vector[vector[vector[double]]] alpha
        bool record_alpha

        vector[vector[vector[double]]] dopa_sum
        bool record_dopa_sum

        vector[vector[vector[double]]] trace
        bool record_trace

        vector[vector[vector[double]]] aux
        bool record_aux

        vector[vector[vector[double]]] dopa_mod
        bool record_dopa_mod

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 52 : Monitor
    cdef cppclass ProjRecorder52 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder52* get_instance(int)

    # Projection 53 : Monitor
    cdef cppclass ProjRecorder53 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder53* get_instance(int)

    # Projection 54 : Monitor
    cdef cppclass ProjRecorder54 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder54* get_instance(int)

    # Projection 55 : Monitor
    cdef cppclass ProjRecorder55 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder55* get_instance(int)

    # Projection 56 : Monitor
    cdef cppclass ProjRecorder56 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder56* get_instance(int)

    # Projection 57 : Monitor
    cdef cppclass ProjRecorder57 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder57* get_instance(int)

    # Projection 58 : Monitor
    cdef cppclass ProjRecorder58 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder58* get_instance(int)

    # Projection 59 : Monitor
    cdef cppclass ProjRecorder59 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder59* get_instance(int)

    # Projection 60 : Monitor
    cdef cppclass ProjRecorder60 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder60* get_instance(int)

    # Projection 61 : Monitor
    cdef cppclass ProjRecorder61 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder61* get_instance(int)

        vector[vector[vector[double]]] aux
        bool record_aux

        vector[vector[vector[double]]] delta
        bool record_delta

        vector[vector[vector[double]]] w
        bool record_w

    # Projection 62 : Monitor
    cdef cppclass ProjRecorder62 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder62* get_instance(int)

    # Projection 63 : Monitor
    cdef cppclass ProjRecorder63 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder63* get_instance(int)

    # Projection 64 : Monitor
    cdef cppclass ProjRecorder64 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder64* get_instance(int)

    # Projection 65 : Monitor
    cdef cppclass ProjRecorder65 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder65* get_instance(int)

    # Projection 66 : Monitor
    cdef cppclass ProjRecorder66 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder66* get_instance(int)

    # Projection 67 : Monitor
    cdef cppclass ProjRecorder67 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder67* get_instance(int)

    # Projection 68 : Monitor
    cdef cppclass ProjRecorder68 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder68* get_instance(int)

    # Projection 69 : Monitor
    cdef cppclass ProjRecorder69 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder69* get_instance(int)

    # Projection 70 : Monitor
    cdef cppclass ProjRecorder70 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder70* get_instance(int)

    # Projection 71 : Monitor
    cdef cppclass ProjRecorder71 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder71* get_instance(int)

    # Projection 72 : Monitor
    cdef cppclass ProjRecorder72 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder72* get_instance(int)

    # Projection 73 : Monitor
    cdef cppclass ProjRecorder73 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder73* get_instance(int)

    # Projection 74 : Monitor
    cdef cppclass ProjRecorder74 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder74* get_instance(int)

    # Projection 75 : Monitor
    cdef cppclass ProjRecorder75 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder75* get_instance(int)

    # Projection 76 : Monitor
    cdef cppclass ProjRecorder76 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder76* get_instance(int)

    # Projection 77 : Monitor
    cdef cppclass ProjRecorder77 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder77* get_instance(int)

    # Projection 78 : Monitor
    cdef cppclass ProjRecorder78 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder78* get_instance(int)

    # Projection 79 : Monitor
    cdef cppclass ProjRecorder79 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder79* get_instance(int)

    # Projection 80 : Monitor
    cdef cppclass ProjRecorder80 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder80* get_instance(int)

    # Projection 81 : Monitor
    cdef cppclass ProjRecorder81 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder81* get_instance(int)

    # Projection 82 : Monitor
    cdef cppclass ProjRecorder82 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder82* get_instance(int)

    # Projection 83 : Monitor
    cdef cppclass ProjRecorder83 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder83* get_instance(int)

    # Projection 84 : Monitor
    cdef cppclass ProjRecorder84 (Monitor):
        @staticmethod
        int create_instance(vector[int], int, int, long)
        @staticmethod
        ProjRecorder84* get_instance(int)


    # Instances

    PopStruct0 pop0
    PopStruct1 pop1
    PopStruct2 pop2
    PopStruct3 pop3
    PopStruct4 pop4
    PopStruct5 pop5
    PopStruct6 pop6
    PopStruct7 pop7
    PopStruct8 pop8
    PopStruct9 pop9
    PopStruct10 pop10
    PopStruct11 pop11
    PopStruct12 pop12
    PopStruct13 pop13
    PopStruct14 pop14
    PopStruct15 pop15
    PopStruct16 pop16
    PopStruct17 pop17
    PopStruct18 pop18
    PopStruct19 pop19
    PopStruct20 pop20
    PopStruct21 pop21
    PopStruct22 pop22
    PopStruct23 pop23
    PopStruct24 pop24
    PopStruct25 pop25
    PopStruct26 pop26
    PopStruct27 pop27
    PopStruct28 pop28
    PopStruct29 pop29

    ProjStruct0 proj0
    ProjStruct1 proj1
    ProjStruct2 proj2
    ProjStruct3 proj3
    ProjStruct4 proj4
    ProjStruct5 proj5
    ProjStruct6 proj6
    ProjStruct7 proj7
    ProjStruct8 proj8
    ProjStruct9 proj9
    ProjStruct10 proj10
    ProjStruct11 proj11
    ProjStruct12 proj12
    ProjStruct13 proj13
    ProjStruct14 proj14
    ProjStruct15 proj15
    ProjStruct16 proj16
    ProjStruct17 proj17
    ProjStruct18 proj18
    ProjStruct19 proj19
    ProjStruct20 proj20
    ProjStruct21 proj21
    ProjStruct22 proj22
    ProjStruct23 proj23
    ProjStruct24 proj24
    ProjStruct25 proj25
    ProjStruct26 proj26
    ProjStruct27 proj27
    ProjStruct28 proj28
    ProjStruct29 proj29
    ProjStruct30 proj30
    ProjStruct31 proj31
    ProjStruct32 proj32
    ProjStruct33 proj33
    ProjStruct34 proj34
    ProjStruct35 proj35
    ProjStruct36 proj36
    ProjStruct37 proj37
    ProjStruct38 proj38
    ProjStruct39 proj39
    ProjStruct40 proj40
    ProjStruct41 proj41
    ProjStruct42 proj42
    ProjStruct43 proj43
    ProjStruct44 proj44
    ProjStruct45 proj45
    ProjStruct46 proj46
    ProjStruct47 proj47
    ProjStruct48 proj48
    ProjStruct49 proj49
    ProjStruct50 proj50
    ProjStruct51 proj51
    ProjStruct52 proj52
    ProjStruct53 proj53
    ProjStruct54 proj54
    ProjStruct55 proj55
    ProjStruct56 proj56
    ProjStruct57 proj57
    ProjStruct58 proj58
    ProjStruct59 proj59
    ProjStruct60 proj60
    ProjStruct61 proj61
    ProjStruct62 proj62
    ProjStruct63 proj63
    ProjStruct64 proj64
    ProjStruct65 proj65
    ProjStruct66 proj66
    ProjStruct67 proj67
    ProjStruct68 proj68
    ProjStruct69 proj69
    ProjStruct70 proj70
    ProjStruct71 proj71
    ProjStruct72 proj72
    ProjStruct73 proj73
    ProjStruct74 proj74
    ProjStruct75 proj75
    ProjStruct76 proj76
    ProjStruct77 proj77
    ProjStruct78 proj78
    ProjStruct79 proj79
    ProjStruct80 proj80
    ProjStruct81 proj81
    ProjStruct82 proj82
    ProjStruct83 proj83
    ProjStruct84 proj84

    # Methods
    void initialize(double)
    void setSeed(long, int, bool)
    void run(int nbSteps) nogil
    int run_until(int steps, vector[int] populations, bool or_and)
    void step()

    # Time
    long getTime()
    void setTime(long)

    # dt
    double getDt()
    void setDt(double dt_)


    # Number of threads
    void setNumberThreads(int, vector[int])


# Profiling (if needed)


# Population wrappers

# Wrapper for population 0 (hippocampus)
@cython.auto_pickle(True)
cdef class pop0_wrapper :

    def __init__(self, size, max_delay):

        pop0.set_size(size)
        pop0.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop0.get_size()
    # Reset the population
    def reset(self):
        pop0.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop0.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop0.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop0.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop0.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop0.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop0.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop0.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop0.size_in_bytes()

    def clear(self):
        return pop0.clear()

# Wrapper for population 1 (hippocampus_activity)
@cython.auto_pickle(True)
cdef class pop1_wrapper :

    def __init__(self, size, max_delay):

        pop1.set_size(size)
        pop1.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop1.get_size()
    # Reset the population
    def reset(self):
        pop1.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop1.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop1.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop1.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop1.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop1.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop1.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop1.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop1.size_in_bytes()

    def clear(self):
        return pop1.clear()

# Wrapper for population 2 (reward_prediction_error)
@cython.auto_pickle(True)
cdef class pop2_wrapper :

    def __init__(self, size, max_delay):

        pop2.set_size(size)
        pop2.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop2.get_size()
    # Reset the population
    def reset(self):
        pop2.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop2.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop2.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop2.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop2.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop2.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop2.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop2.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop2.size_in_bytes()

    def clear(self):
        return pop2.clear()

# Wrapper for population 3 (nacc_shell_d1)
@cython.auto_pickle(True)
cdef class pop3_wrapper :

    def __init__(self, size, max_delay):

        pop3.set_size(size)
        pop3.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop3.get_size()
    # Reset the population
    def reset(self):
        pop3.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop3.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop3.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop3.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop3.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop3.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop3.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop3.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop3.size_in_bytes()

    def clear(self):
        return pop3.clear()

# Wrapper for population 4 (nacc_shell_d2)
@cython.auto_pickle(True)
cdef class pop4_wrapper :

    def __init__(self, size, max_delay):

        pop4.set_size(size)
        pop4.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop4.get_size()
    # Reset the population
    def reset(self):
        pop4.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop4.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop4.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop4.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop4.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop4.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop4.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop4.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop4.size_in_bytes()

    def clear(self):
        return pop4.clear()

# Wrapper for population 5 (stn_ventral)
@cython.auto_pickle(True)
cdef class pop5_wrapper :

    def __init__(self, size, max_delay):

        pop5.set_size(size)
        pop5.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop5.get_size()
    # Reset the population
    def reset(self):
        pop5.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop5.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop5.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop5.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop5.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop5.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop5.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop5.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop5.size_in_bytes()

    def clear(self):
        return pop5.clear()

# Wrapper for population 6 (vp_direct)
@cython.auto_pickle(True)
cdef class pop6_wrapper :

    def __init__(self, size, max_delay):

        pop6.set_size(size)
        pop6.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop6.get_size()
    # Reset the population
    def reset(self):
        pop6.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop6.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop6.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop6.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop6.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop6.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop6.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop6.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop6.size_in_bytes()

    def clear(self):
        return pop6.clear()

# Wrapper for population 7 (vp_indirect)
@cython.auto_pickle(True)
cdef class pop7_wrapper :

    def __init__(self, size, max_delay):

        pop7.set_size(size)
        pop7.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop7.get_size()
    # Reset the population
    def reset(self):
        pop7.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop7.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop7.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop7.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop7.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop7.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop7.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop7.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop7.size_in_bytes()

    def clear(self):
        return pop7.clear()

# Wrapper for population 8 (ventral_tegmental_area)
@cython.auto_pickle(True)
cdef class pop8_wrapper :

    def __init__(self, size, max_delay):

        pop8.set_size(size)
        pop8.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop8.get_size()
    # Reset the population
    def reset(self):
        pop8.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop8.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop8.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop8.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop8.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop8.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop8.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop8.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop8.size_in_bytes()

    def clear(self):
        return pop8.clear()

# Wrapper for population 9 (pptn_ventral)
@cython.auto_pickle(True)
cdef class pop9_wrapper :

    def __init__(self, size, max_delay):

        pop9.set_size(size)
        pop9.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop9.get_size()
    # Reset the population
    def reset(self):
        pop9.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop9.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop9.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop9.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop9.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop9.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop9.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop9.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop9.size_in_bytes()

    def clear(self):
        return pop9.clear()

# Wrapper for population 10 (cortical_feedback_ventral)
@cython.auto_pickle(True)
cdef class pop10_wrapper :

    def __init__(self, size, max_delay):

        pop10.set_size(size)
        pop10.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop10.get_size()
    # Reset the population
    def reset(self):
        pop10.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop10.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop10.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop10.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop10.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop10.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop10.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop10.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop10.size_in_bytes()

    def clear(self):
        return pop10.clear()

# Wrapper for population 11 (thalamus_ventral)
@cython.auto_pickle(True)
cdef class pop11_wrapper :

    def __init__(self, size, max_delay):

        pop11.set_size(size)
        pop11.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop11.get_size()
    # Reset the population
    def reset(self):
        pop11.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop11.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop11.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop11.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop11.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop11.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop11.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop11.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop11.size_in_bytes()

    def clear(self):
        return pop11.clear()

# Wrapper for population 12 (medial_pfc)
@cython.auto_pickle(True)
cdef class pop12_wrapper :

    def __init__(self, size, max_delay):

        pop12.set_size(size)
        pop12.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop12.get_size()
    # Reset the population
    def reset(self):
        pop12.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop12.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop12.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop12.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop12.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop12.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop12.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop12.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop12.size_in_bytes()

    def clear(self):
        return pop12.clear()

# Wrapper for population 13 (nacc_core)
@cython.auto_pickle(True)
cdef class pop13_wrapper :

    def __init__(self, size, max_delay):

        pop13.set_size(size)
        pop13.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop13.get_size()
    # Reset the population
    def reset(self):
        pop13.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop13.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop13.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop13.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop13.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop13.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop13.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop13.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop13.size_in_bytes()

    def clear(self):
        return pop13.clear()

# Wrapper for population 14 (substantia_nigra_pars_reticulata)
@cython.auto_pickle(True)
cdef class pop14_wrapper :

    def __init__(self, size, max_delay):

        pop14.set_size(size)
        pop14.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop14.get_size()
    # Reset the population
    def reset(self):
        pop14.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop14.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop14.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop14.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop14.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop14.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop14.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop14.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop14.size_in_bytes()

    def clear(self):
        return pop14.clear()

# Wrapper for population 15 (visual_input)
@cython.auto_pickle(True)
cdef class pop15_wrapper :

    def __init__(self, size, max_delay):

        pop15.set_size(size)
        pop15.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop15.get_size()
    # Reset the population
    def reset(self):
        pop15.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop15.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop15.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop15.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop15.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop15.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop15.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop15.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop15.size_in_bytes()

    def clear(self):
        return pop15.clear()

# Wrapper for population 16 (striatum_d1_dorsomedial)
@cython.auto_pickle(True)
cdef class pop16_wrapper :

    def __init__(self, size, max_delay):

        pop16.set_size(size)
        pop16.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop16.get_size()
    # Reset the population
    def reset(self):
        pop16.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop16.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop16.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop16.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop16.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop16.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop16.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop16.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop16.size_in_bytes()

    def clear(self):
        return pop16.clear()

# Wrapper for population 17 (striatum_d2_dorsomedial)
@cython.auto_pickle(True)
cdef class pop17_wrapper :

    def __init__(self, size, max_delay):

        pop17.set_size(size)
        pop17.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop17.get_size()
    # Reset the population
    def reset(self):
        pop17.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop17.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop17.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop17.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop17.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop17.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop17.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop17.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop17.size_in_bytes()

    def clear(self):
        return pop17.clear()

# Wrapper for population 18 (stn_dorsomedial)
@cython.auto_pickle(True)
cdef class pop18_wrapper :

    def __init__(self, size, max_delay):

        pop18.set_size(size)
        pop18.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop18.get_size()
    # Reset the population
    def reset(self):
        pop18.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop18.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop18.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop18.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop18.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop18.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop18.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop18.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop18.size_in_bytes()

    def clear(self):
        return pop18.clear()

# Wrapper for population 19 (gpi_dorsomedial)
@cython.auto_pickle(True)
cdef class pop19_wrapper :

    def __init__(self, size, max_delay):

        pop19.set_size(size)
        pop19.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop19.get_size()
    # Reset the population
    def reset(self):
        pop19.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop19.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop19.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop19.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop19.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop19.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop19.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop19.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop19.size_in_bytes()

    def clear(self):
        return pop19.clear()

# Wrapper for population 20 (gpe_dorsomedial)
@cython.auto_pickle(True)
cdef class pop20_wrapper :

    def __init__(self, size, max_delay):

        pop20.set_size(size)
        pop20.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop20.get_size()
    # Reset the population
    def reset(self):
        pop20.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop20.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop20.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop20.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop20.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop20.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop20.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop20.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop20.size_in_bytes()

    def clear(self):
        return pop20.clear()

# Wrapper for population 21 (substantia_nigra_pars_compacta)
@cython.auto_pickle(True)
cdef class pop21_wrapper :

    def __init__(self, size, max_delay):

        pop21.set_size(size)
        pop21.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop21.get_size()
    # Reset the population
    def reset(self):
        pop21.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop21.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop21.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop21.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop21.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop21.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop21.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop21.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop21.size_in_bytes()

    def clear(self):
        return pop21.clear()

# Wrapper for population 22 (pptn_dorsomedial)
@cython.auto_pickle(True)
cdef class pop22_wrapper :

    def __init__(self, size, max_delay):

        pop22.set_size(size)
        pop22.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop22.get_size()
    # Reset the population
    def reset(self):
        pop22.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop22.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop22.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop22.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop22.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop22.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop22.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop22.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop22.size_in_bytes()

    def clear(self):
        return pop22.clear()

# Wrapper for population 23 (cortical_feedback_dorsomedial)
@cython.auto_pickle(True)
cdef class pop23_wrapper :

    def __init__(self, size, max_delay):

        pop23.set_size(size)
        pop23.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop23.get_size()
    # Reset the population
    def reset(self):
        pop23.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop23.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop23.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop23.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop23.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop23.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop23.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop23.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop23.size_in_bytes()

    def clear(self):
        return pop23.clear()

# Wrapper for population 24 (thalamus_dorsomedial)
@cython.auto_pickle(True)
cdef class pop24_wrapper :

    def __init__(self, size, max_delay):

        pop24.set_size(size)
        pop24.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop24.get_size()
    # Reset the population
    def reset(self):
        pop24.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop24.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop24.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop24.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop24.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop24.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop24.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop24.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop24.size_in_bytes()

    def clear(self):
        return pop24.clear()

# Wrapper for population 25 (dorsolateral_pfc)
@cython.auto_pickle(True)
cdef class pop25_wrapper :

    def __init__(self, size, max_delay):

        pop25.set_size(size)
        pop25.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop25.get_size()
    # Reset the population
    def reset(self):
        pop25.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop25.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop25.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop25.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop25.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop25.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop25.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop25.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop25.size_in_bytes()

    def clear(self):
        return pop25.clear()

# Wrapper for population 26 (striatum_d1_dorsolateral)
@cython.auto_pickle(True)
cdef class pop26_wrapper :

    def __init__(self, size, max_delay):

        pop26.set_size(size)
        pop26.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop26.get_size()
    # Reset the population
    def reset(self):
        pop26.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop26.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop26.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop26.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop26.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop26.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop26.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop26.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop26.size_in_bytes()

    def clear(self):
        return pop26.clear()

# Wrapper for population 27 (gpi_dorsolateral)
@cython.auto_pickle(True)
cdef class pop27_wrapper :

    def __init__(self, size, max_delay):

        pop27.set_size(size)
        pop27.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop27.get_size()
    # Reset the population
    def reset(self):
        pop27.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop27.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop27.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop27.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop27.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop27.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop27.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop27.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop27.size_in_bytes()

    def clear(self):
        return pop27.clear()

# Wrapper for population 28 (thalamus_dorsolateral)
@cython.auto_pickle(True)
cdef class pop28_wrapper :

    def __init__(self, size, max_delay):

        pop28.set_size(size)
        pop28.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop28.get_size()
    # Reset the population
    def reset(self):
        pop28.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop28.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop28.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop28.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop28.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop28.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop28.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop28.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop28.size_in_bytes()

    def clear(self):
        return pop28.clear()

# Wrapper for population 29 (premotor)
@cython.auto_pickle(True)
cdef class pop29_wrapper :

    def __init__(self, size, max_delay):

        pop29.set_size(size)
        pop29.set_max_delay(max_delay)
    # Number of neurons
    property size:
        def __get__(self):
            return pop29.get_size()
    # Reset the population
    def reset(self):
        pop29.reset()
    # Set the maximum delay of outgoing projections
    def set_max_delay(self, val):
        pop29.set_max_delay(val)
    # Updates the maximum delay of outgoing projections and rebuilds the arrays
    def update_max_delay(self, val):
        pop29.update_max_delay(val)
    # Allows the population to compute
    def activate(self, bool val):
        pop29.set_active(val)


    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return np.array(pop29.get_local_attribute_all_double(cpp_string))


    def get_local_attribute(self, name, rk, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return pop29.get_local_attribute_double(cpp_string, rk)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop29.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute(self, name, rk, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            pop29.set_local_attribute_double(cpp_string, rk, value)







    # memory management
    def size_in_bytes(self):
        return pop29.size_in_bytes()

    def clear(self):
        return pop29.clear()


# Projection wrappers

# Wrapper for projection 0
@cython.auto_pickle(True)
cdef class proj0_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj0.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj0.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj0.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj0._transmission
    def _set_transmission(self, bool l):
        proj0._transmission = l

    # Update flag
    def _get_update(self):
        return proj0._update
    def _set_update(self, bool l):
        proj0._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj0._plasticity
    def _set_plasticity(self, bool l):
        proj0._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj0._update_period
    def _set_update_period(self, int l):
        proj0._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj0._update_offset
    def _set_update_offset(self, long l):
        proj0._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj0.get_post_rank()
    def pre_rank_all(self):
        return proj0.get_pre_ranks()
    def pre_rank(self, int n):
        return proj0.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj0.nb_dendrites()
    def nb_synapses(self):
        return proj0.nb_synapses()
    def dendrite_size(self, int n):
        return proj0.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj0.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj0.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj0.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj0.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj0.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj0.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj0.size_in_bytes()

    def clear(self):
        return proj0.clear()

# Wrapper for projection 1
@cython.auto_pickle(True)
cdef class proj1_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj1.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj1.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj1.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj1._transmission
    def _set_transmission(self, bool l):
        proj1._transmission = l

    # Update flag
    def _get_update(self):
        return proj1._update
    def _set_update(self, bool l):
        proj1._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj1._plasticity
    def _set_plasticity(self, bool l):
        proj1._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj1._update_period
    def _set_update_period(self, int l):
        proj1._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj1._update_offset
    def _set_update_offset(self, long l):
        proj1._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj1.get_post_rank()
    def pre_rank_all(self):
        return proj1.get_pre_ranks()
    def pre_rank(self, int n):
        return proj1.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj1.nb_dendrites()
    def nb_synapses(self):
        return proj1.nb_synapses()
    def dendrite_size(self, int n):
        return proj1.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj1.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj1.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj1.size_in_bytes()

    def clear(self):
        return proj1.clear()

# Wrapper for projection 2
@cython.auto_pickle(True)
cdef class proj2_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj2.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj2.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj2.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj2._transmission
    def _set_transmission(self, bool l):
        proj2._transmission = l

    # Update flag
    def _get_update(self):
        return proj2._update
    def _set_update(self, bool l):
        proj2._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj2._plasticity
    def _set_plasticity(self, bool l):
        proj2._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj2._update_period
    def _set_update_period(self, int l):
        proj2._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj2._update_offset
    def _set_update_offset(self, long l):
        proj2._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj2.get_post_rank()
    def pre_rank_all(self):
        return proj2.get_pre_ranks()
    def pre_rank(self, int n):
        return proj2.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj2.nb_dendrites()
    def nb_synapses(self):
        return proj2.nb_synapses()
    def dendrite_size(self, int n):
        return proj2.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj2.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj2.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj2.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj2.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj2.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj2.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj2.size_in_bytes()

    def clear(self):
        return proj2.clear()

# Wrapper for projection 3
@cython.auto_pickle(True)
cdef class proj3_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj3.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj3.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj3.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj3._transmission
    def _set_transmission(self, bool l):
        proj3._transmission = l

    # Update flag
    def _get_update(self):
        return proj3._update
    def _set_update(self, bool l):
        proj3._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj3._plasticity
    def _set_plasticity(self, bool l):
        proj3._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj3._update_period
    def _set_update_period(self, int l):
        proj3._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj3._update_offset
    def _set_update_offset(self, long l):
        proj3._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj3.get_post_rank()
    def pre_rank_all(self):
        return proj3.get_pre_ranks()
    def pre_rank(self, int n):
        return proj3.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj3.nb_dendrites()
    def nb_synapses(self):
        return proj3.nb_synapses()
    def dendrite_size(self, int n):
        return proj3.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj3.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj3.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj3.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj3.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj3.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj3.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)


    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj3.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj3.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj3.size_in_bytes()

    def clear(self):
        return proj3.clear()

# Wrapper for projection 4
@cython.auto_pickle(True)
cdef class proj4_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj4.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj4.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj4.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj4._transmission
    def _set_transmission(self, bool l):
        proj4._transmission = l

    # Update flag
    def _get_update(self):
        return proj4._update
    def _set_update(self, bool l):
        proj4._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj4._plasticity
    def _set_plasticity(self, bool l):
        proj4._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj4._update_period
    def _set_update_period(self, int l):
        proj4._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj4._update_offset
    def _set_update_offset(self, long l):
        proj4._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj4.get_post_rank()
    def pre_rank_all(self):
        return proj4.get_pre_ranks()
    def pre_rank(self, int n):
        return proj4.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj4.nb_dendrites()
    def nb_synapses(self):
        return proj4.nb_synapses()
    def dendrite_size(self, int n):
        return proj4.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj4.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj4.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj4.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj4.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj4.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj4.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)


    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj4.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj4.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj4.size_in_bytes()

    def clear(self):
        return proj4.clear()

# Wrapper for projection 5
@cython.auto_pickle(True)
cdef class proj5_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj5.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj5.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj5.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj5._transmission
    def _set_transmission(self, bool l):
        proj5._transmission = l

    # Update flag
    def _get_update(self):
        return proj5._update
    def _set_update(self, bool l):
        proj5._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj5._plasticity
    def _set_plasticity(self, bool l):
        proj5._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj5._update_period
    def _set_update_period(self, int l):
        proj5._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj5._update_offset
    def _set_update_offset(self, long l):
        proj5._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj5.get_post_rank()
    def pre_rank_all(self):
        return proj5.get_pre_ranks()
    def pre_rank(self, int n):
        return proj5.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj5.nb_dendrites()
    def nb_synapses(self):
        return proj5.nb_synapses()
    def dendrite_size(self, int n):
        return proj5.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj5.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj5.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj5.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj5.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj5.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj5.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)


    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj5.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj5.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj5.size_in_bytes()

    def clear(self):
        return proj5.clear()

# Wrapper for projection 6
@cython.auto_pickle(True)
cdef class proj6_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj6.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj6.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj6.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj6._transmission
    def _set_transmission(self, bool l):
        proj6._transmission = l

    # Update flag
    def _get_update(self):
        return proj6._update
    def _set_update(self, bool l):
        proj6._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj6._plasticity
    def _set_plasticity(self, bool l):
        proj6._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj6._update_period
    def _set_update_period(self, int l):
        proj6._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj6._update_offset
    def _set_update_offset(self, long l):
        proj6._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj6.get_post_rank()
    def pre_rank_all(self):
        return proj6.get_pre_ranks()
    def pre_rank(self, int n):
        return proj6.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj6.nb_dendrites()
    def nb_synapses(self):
        return proj6.nb_synapses()
    def dendrite_size(self, int n):
        return proj6.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj6.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj6.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj6.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj6.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj6.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj6.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)


    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj6.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj6.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj6.size_in_bytes()

    def clear(self):
        return proj6.clear()

# Wrapper for projection 7
@cython.auto_pickle(True)
cdef class proj7_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj7.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj7.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj7.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj7._transmission
    def _set_transmission(self, bool l):
        proj7._transmission = l

    # Update flag
    def _get_update(self):
        return proj7._update
    def _set_update(self, bool l):
        proj7._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj7._plasticity
    def _set_plasticity(self, bool l):
        proj7._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj7._update_period
    def _set_update_period(self, int l):
        proj7._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj7._update_offset
    def _set_update_offset(self, long l):
        proj7._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj7.get_post_rank()
    def pre_rank_all(self):
        return proj7.get_pre_ranks()
    def pre_rank(self, int n):
        return proj7.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj7.nb_dendrites()
    def nb_synapses(self):
        return proj7.nb_synapses()
    def dendrite_size(self, int n):
        return proj7.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj7.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj7.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj7.size_in_bytes()

    def clear(self):
        return proj7.clear()

# Wrapper for projection 8
@cython.auto_pickle(True)
cdef class proj8_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj8.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj8.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj8.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj8._transmission
    def _set_transmission(self, bool l):
        proj8._transmission = l

    # Update flag
    def _get_update(self):
        return proj8._update
    def _set_update(self, bool l):
        proj8._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj8._plasticity
    def _set_plasticity(self, bool l):
        proj8._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj8._update_period
    def _set_update_period(self, int l):
        proj8._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj8._update_offset
    def _set_update_offset(self, long l):
        proj8._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj8.get_post_rank()
    def pre_rank_all(self):
        return proj8.get_pre_ranks()
    def pre_rank(self, int n):
        return proj8.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj8.nb_dendrites()
    def nb_synapses(self):
        return proj8.nb_synapses()
    def dendrite_size(self, int n):
        return proj8.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj8.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj8.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj8.size_in_bytes()

    def clear(self):
        return proj8.clear()

# Wrapper for projection 9
@cython.auto_pickle(True)
cdef class proj9_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj9.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj9.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj9.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj9._transmission
    def _set_transmission(self, bool l):
        proj9._transmission = l

    # Update flag
    def _get_update(self):
        return proj9._update
    def _set_update(self, bool l):
        proj9._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj9._plasticity
    def _set_plasticity(self, bool l):
        proj9._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj9._update_period
    def _set_update_period(self, int l):
        proj9._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj9._update_offset
    def _set_update_offset(self, long l):
        proj9._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj9.get_post_rank()
    def pre_rank_all(self):
        return proj9.get_pre_ranks()
    def pre_rank(self, int n):
        return proj9.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj9.nb_dendrites()
    def nb_synapses(self):
        return proj9.nb_synapses()
    def dendrite_size(self, int n):
        return proj9.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj9.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj9.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj9.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj9.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj9.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj9.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj9.size_in_bytes()

    def clear(self):
        return proj9.clear()

# Wrapper for projection 10
@cython.auto_pickle(True)
cdef class proj10_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj10.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj10.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj10.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj10._transmission
    def _set_transmission(self, bool l):
        proj10._transmission = l

    # Update flag
    def _get_update(self):
        return proj10._update
    def _set_update(self, bool l):
        proj10._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj10._plasticity
    def _set_plasticity(self, bool l):
        proj10._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj10._update_period
    def _set_update_period(self, int l):
        proj10._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj10._update_offset
    def _set_update_offset(self, long l):
        proj10._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj10.get_post_rank()
    def pre_rank_all(self):
        return proj10.get_pre_ranks()
    def pre_rank(self, int n):
        return proj10.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj10.nb_dendrites()
    def nb_synapses(self):
        return proj10.nb_synapses()
    def dendrite_size(self, int n):
        return proj10.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj10.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj10.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj10.size_in_bytes()

    def clear(self):
        return proj10.clear()

# Wrapper for projection 11
@cython.auto_pickle(True)
cdef class proj11_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj11.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj11.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj11.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj11._transmission
    def _set_transmission(self, bool l):
        proj11._transmission = l

    # Update flag
    def _get_update(self):
        return proj11._update
    def _set_update(self, bool l):
        proj11._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj11._plasticity
    def _set_plasticity(self, bool l):
        proj11._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj11._update_period
    def _set_update_period(self, int l):
        proj11._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj11._update_offset
    def _set_update_offset(self, long l):
        proj11._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj11.get_post_rank()
    def pre_rank_all(self):
        return proj11.get_pre_ranks()
    def pre_rank(self, int n):
        return proj11.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj11.nb_dendrites()
    def nb_synapses(self):
        return proj11.nb_synapses()
    def dendrite_size(self, int n):
        return proj11.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj11.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj11.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj11.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj11.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj11.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj11.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj11.size_in_bytes()

    def clear(self):
        return proj11.clear()

# Wrapper for projection 12
@cython.auto_pickle(True)
cdef class proj12_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj12.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj12.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj12.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj12._transmission
    def _set_transmission(self, bool l):
        proj12._transmission = l

    # Update flag
    def _get_update(self):
        return proj12._update
    def _set_update(self, bool l):
        proj12._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj12._plasticity
    def _set_plasticity(self, bool l):
        proj12._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj12._update_period
    def _set_update_period(self, int l):
        proj12._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj12._update_offset
    def _set_update_offset(self, long l):
        proj12._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj12.get_post_rank()
    def pre_rank_all(self):
        return proj12.get_pre_ranks()
    def pre_rank(self, int n):
        return proj12.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj12.nb_dendrites()
    def nb_synapses(self):
        return proj12.nb_synapses()
    def dendrite_size(self, int n):
        return proj12.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj12.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj12.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj12.size_in_bytes()

    def clear(self):
        return proj12.clear()

# Wrapper for projection 13
@cython.auto_pickle(True)
cdef class proj13_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj13.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj13.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj13.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj13._transmission
    def _set_transmission(self, bool l):
        proj13._transmission = l

    # Update flag
    def _get_update(self):
        return proj13._update
    def _set_update(self, bool l):
        proj13._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj13._plasticity
    def _set_plasticity(self, bool l):
        proj13._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj13._update_period
    def _set_update_period(self, int l):
        proj13._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj13._update_offset
    def _set_update_offset(self, long l):
        proj13._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj13.get_post_rank()
    def pre_rank_all(self):
        return proj13.get_pre_ranks()
    def pre_rank(self, int n):
        return proj13.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj13.nb_dendrites()
    def nb_synapses(self):
        return proj13.nb_synapses()
    def dendrite_size(self, int n):
        return proj13.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj13.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj13.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj13.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj13.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj13.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj13.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj13.size_in_bytes()

    def clear(self):
        return proj13.clear()

# Wrapper for projection 14
@cython.auto_pickle(True)
cdef class proj14_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj14.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj14.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj14.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj14._transmission
    def _set_transmission(self, bool l):
        proj14._transmission = l

    # Update flag
    def _get_update(self):
        return proj14._update
    def _set_update(self, bool l):
        proj14._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj14._plasticity
    def _set_plasticity(self, bool l):
        proj14._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj14._update_period
    def _set_update_period(self, int l):
        proj14._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj14._update_offset
    def _set_update_offset(self, long l):
        proj14._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj14.get_post_rank()
    def pre_rank_all(self):
        return proj14.get_pre_ranks()
    def pre_rank(self, int n):
        return proj14.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj14.nb_dendrites()
    def nb_synapses(self):
        return proj14.nb_synapses()
    def dendrite_size(self, int n):
        return proj14.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj14.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj14.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj14.size_in_bytes()

    def clear(self):
        return proj14.clear()

# Wrapper for projection 15
@cython.auto_pickle(True)
cdef class proj15_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj15.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj15.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj15.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj15._transmission
    def _set_transmission(self, bool l):
        proj15._transmission = l

    # Update flag
    def _get_update(self):
        return proj15._update
    def _set_update(self, bool l):
        proj15._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj15._plasticity
    def _set_plasticity(self, bool l):
        proj15._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj15._update_period
    def _set_update_period(self, int l):
        proj15._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj15._update_offset
    def _set_update_offset(self, long l):
        proj15._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj15.get_post_rank()
    def pre_rank_all(self):
        return proj15.get_pre_ranks()
    def pre_rank(self, int n):
        return proj15.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj15.nb_dendrites()
    def nb_synapses(self):
        return proj15.nb_synapses()
    def dendrite_size(self, int n):
        return proj15.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj15.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj15.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj15.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj15.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj15.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj15.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj15.size_in_bytes()

    def clear(self):
        return proj15.clear()

# Wrapper for projection 16
@cython.auto_pickle(True)
cdef class proj16_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj16.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj16.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj16.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj16._transmission
    def _set_transmission(self, bool l):
        proj16._transmission = l

    # Update flag
    def _get_update(self):
        return proj16._update
    def _set_update(self, bool l):
        proj16._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj16._plasticity
    def _set_plasticity(self, bool l):
        proj16._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj16._update_period
    def _set_update_period(self, int l):
        proj16._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj16._update_offset
    def _set_update_offset(self, long l):
        proj16._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj16.get_post_rank()
    def pre_rank_all(self):
        return proj16.get_pre_ranks()
    def pre_rank(self, int n):
        return proj16.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj16.nb_dendrites()
    def nb_synapses(self):
        return proj16.nb_synapses()
    def dendrite_size(self, int n):
        return proj16.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj16.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj16.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj16.size_in_bytes()

    def clear(self):
        return proj16.clear()

# Wrapper for projection 17
@cython.auto_pickle(True)
cdef class proj17_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj17.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj17.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj17.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj17._transmission
    def _set_transmission(self, bool l):
        proj17._transmission = l

    # Update flag
    def _get_update(self):
        return proj17._update
    def _set_update(self, bool l):
        proj17._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj17._plasticity
    def _set_plasticity(self, bool l):
        proj17._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj17._update_period
    def _set_update_period(self, int l):
        proj17._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj17._update_offset
    def _set_update_offset(self, long l):
        proj17._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj17.get_post_rank()
    def pre_rank_all(self):
        return proj17.get_pre_ranks()
    def pre_rank(self, int n):
        return proj17.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj17.nb_dendrites()
    def nb_synapses(self):
        return proj17.nb_synapses()
    def dendrite_size(self, int n):
        return proj17.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj17.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj17.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj17.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj17.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj17.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj17.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj17.size_in_bytes()

    def clear(self):
        return proj17.clear()

# Wrapper for projection 18
@cython.auto_pickle(True)
cdef class proj18_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj18.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj18.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj18.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj18._transmission
    def _set_transmission(self, bool l):
        proj18._transmission = l

    # Update flag
    def _get_update(self):
        return proj18._update
    def _set_update(self, bool l):
        proj18._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj18._plasticity
    def _set_plasticity(self, bool l):
        proj18._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj18._update_period
    def _set_update_period(self, int l):
        proj18._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj18._update_offset
    def _set_update_offset(self, long l):
        proj18._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj18.get_post_rank()
    def pre_rank_all(self):
        return proj18.get_pre_ranks()
    def pre_rank(self, int n):
        return proj18.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj18.nb_dendrites()
    def nb_synapses(self):
        return proj18.nb_synapses()
    def dendrite_size(self, int n):
        return proj18.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj18.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj18.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj18.size_in_bytes()

    def clear(self):
        return proj18.clear()

# Wrapper for projection 19
@cython.auto_pickle(True)
cdef class proj19_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj19.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj19.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj19.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj19._transmission
    def _set_transmission(self, bool l):
        proj19._transmission = l

    # Update flag
    def _get_update(self):
        return proj19._update
    def _set_update(self, bool l):
        proj19._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj19._plasticity
    def _set_plasticity(self, bool l):
        proj19._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj19._update_period
    def _set_update_period(self, int l):
        proj19._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj19._update_offset
    def _set_update_offset(self, long l):
        proj19._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj19.get_post_rank()
    def pre_rank_all(self):
        return proj19.get_pre_ranks()
    def pre_rank(self, int n):
        return proj19.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj19.nb_dendrites()
    def nb_synapses(self):
        return proj19.nb_synapses()
    def dendrite_size(self, int n):
        return proj19.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj19.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj19.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj19.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj19.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj19.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj19.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj19.size_in_bytes()

    def clear(self):
        return proj19.clear()

# Wrapper for projection 20
@cython.auto_pickle(True)
cdef class proj20_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj20.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj20.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj20.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj20._transmission
    def _set_transmission(self, bool l):
        proj20._transmission = l

    # Update flag
    def _get_update(self):
        return proj20._update
    def _set_update(self, bool l):
        proj20._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj20._plasticity
    def _set_plasticity(self, bool l):
        proj20._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj20._update_period
    def _set_update_period(self, int l):
        proj20._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj20._update_offset
    def _set_update_offset(self, long l):
        proj20._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj20.get_post_rank()
    def pre_rank_all(self):
        return proj20.get_pre_ranks()
    def pre_rank(self, int n):
        return proj20.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj20.nb_dendrites()
    def nb_synapses(self):
        return proj20.nb_synapses()
    def dendrite_size(self, int n):
        return proj20.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj20.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj20.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj20.size_in_bytes()

    def clear(self):
        return proj20.clear()

# Wrapper for projection 21
@cython.auto_pickle(True)
cdef class proj21_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj21.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj21.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj21.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj21._transmission
    def _set_transmission(self, bool l):
        proj21._transmission = l

    # Update flag
    def _get_update(self):
        return proj21._update
    def _set_update(self, bool l):
        proj21._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj21._plasticity
    def _set_plasticity(self, bool l):
        proj21._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj21._update_period
    def _set_update_period(self, int l):
        proj21._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj21._update_offset
    def _set_update_offset(self, long l):
        proj21._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj21.get_post_rank()
    def pre_rank_all(self):
        return proj21.get_pre_ranks()
    def pre_rank(self, int n):
        return proj21.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj21.nb_dendrites()
    def nb_synapses(self):
        return proj21.nb_synapses()
    def dendrite_size(self, int n):
        return proj21.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj21.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj21.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj21.size_in_bytes()

    def clear(self):
        return proj21.clear()

# Wrapper for projection 22
@cython.auto_pickle(True)
cdef class proj22_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj22.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj22.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj22.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj22._transmission
    def _set_transmission(self, bool l):
        proj22._transmission = l

    # Update flag
    def _get_update(self):
        return proj22._update
    def _set_update(self, bool l):
        proj22._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj22._plasticity
    def _set_plasticity(self, bool l):
        proj22._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj22._update_period
    def _set_update_period(self, int l):
        proj22._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj22._update_offset
    def _set_update_offset(self, long l):
        proj22._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj22.get_post_rank()
    def pre_rank_all(self):
        return proj22.get_pre_ranks()
    def pre_rank(self, int n):
        return proj22.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj22.nb_dendrites()
    def nb_synapses(self):
        return proj22.nb_synapses()
    def dendrite_size(self, int n):
        return proj22.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj22.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj22.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj22.size_in_bytes()

    def clear(self):
        return proj22.clear()

# Wrapper for projection 23
@cython.auto_pickle(True)
cdef class proj23_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj23.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj23.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj23.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj23._transmission
    def _set_transmission(self, bool l):
        proj23._transmission = l

    # Update flag
    def _get_update(self):
        return proj23._update
    def _set_update(self, bool l):
        proj23._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj23._plasticity
    def _set_plasticity(self, bool l):
        proj23._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj23._update_period
    def _set_update_period(self, int l):
        proj23._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj23._update_offset
    def _set_update_offset(self, long l):
        proj23._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj23.get_post_rank()
    def pre_rank_all(self):
        return proj23.get_pre_ranks()
    def pre_rank(self, int n):
        return proj23.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj23.nb_dendrites()
    def nb_synapses(self):
        return proj23.nb_synapses()
    def dendrite_size(self, int n):
        return proj23.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj23.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj23.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj23.size_in_bytes()

    def clear(self):
        return proj23.clear()

# Wrapper for projection 24
@cython.auto_pickle(True)
cdef class proj24_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj24.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj24.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj24.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj24._transmission
    def _set_transmission(self, bool l):
        proj24._transmission = l

    # Update flag
    def _get_update(self):
        return proj24._update
    def _set_update(self, bool l):
        proj24._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj24._plasticity
    def _set_plasticity(self, bool l):
        proj24._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj24._update_period
    def _set_update_period(self, int l):
        proj24._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj24._update_offset
    def _set_update_offset(self, long l):
        proj24._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj24.get_post_rank()
    def pre_rank_all(self):
        return proj24.get_pre_ranks()
    def pre_rank(self, int n):
        return proj24.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj24.nb_dendrites()
    def nb_synapses(self):
        return proj24.nb_synapses()
    def dendrite_size(self, int n):
        return proj24.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj24.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj24.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj24.size_in_bytes()

    def clear(self):
        return proj24.clear()

# Wrapper for projection 25
@cython.auto_pickle(True)
cdef class proj25_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj25.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj25.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj25.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj25._transmission
    def _set_transmission(self, bool l):
        proj25._transmission = l

    # Update flag
    def _get_update(self):
        return proj25._update
    def _set_update(self, bool l):
        proj25._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj25._plasticity
    def _set_plasticity(self, bool l):
        proj25._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj25._update_period
    def _set_update_period(self, int l):
        proj25._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj25._update_offset
    def _set_update_offset(self, long l):
        proj25._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj25.get_post_rank()
    def pre_rank_all(self):
        return proj25.get_pre_ranks()
    def pre_rank(self, int n):
        return proj25.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj25.nb_dendrites()
    def nb_synapses(self):
        return proj25.nb_synapses()
    def dendrite_size(self, int n):
        return proj25.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj25.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj25.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj25.size_in_bytes()

    def clear(self):
        return proj25.clear()

# Wrapper for projection 26
@cython.auto_pickle(True)
cdef class proj26_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj26.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj26.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj26.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj26._transmission
    def _set_transmission(self, bool l):
        proj26._transmission = l

    # Update flag
    def _get_update(self):
        return proj26._update
    def _set_update(self, bool l):
        proj26._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj26._plasticity
    def _set_plasticity(self, bool l):
        proj26._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj26._update_period
    def _set_update_period(self, int l):
        proj26._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj26._update_offset
    def _set_update_offset(self, long l):
        proj26._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj26.get_post_rank()
    def pre_rank_all(self):
        return proj26.get_pre_ranks()
    def pre_rank(self, int n):
        return proj26.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj26.nb_dendrites()
    def nb_synapses(self):
        return proj26.nb_synapses()
    def dendrite_size(self, int n):
        return proj26.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj26.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj26.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj26.size_in_bytes()

    def clear(self):
        return proj26.clear()

# Wrapper for projection 27
@cython.auto_pickle(True)
cdef class proj27_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj27.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj27.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj27.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj27._transmission
    def _set_transmission(self, bool l):
        proj27._transmission = l

    # Update flag
    def _get_update(self):
        return proj27._update
    def _set_update(self, bool l):
        proj27._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj27._plasticity
    def _set_plasticity(self, bool l):
        proj27._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj27._update_period
    def _set_update_period(self, int l):
        proj27._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj27._update_offset
    def _set_update_offset(self, long l):
        proj27._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj27.get_post_rank()
    def pre_rank_all(self):
        return proj27.get_pre_ranks()
    def pre_rank(self, int n):
        return proj27.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj27.nb_dendrites()
    def nb_synapses(self):
        return proj27.nb_synapses()
    def dendrite_size(self, int n):
        return proj27.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj27.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj27.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj27.size_in_bytes()

    def clear(self):
        return proj27.clear()

# Wrapper for projection 28
@cython.auto_pickle(True)
cdef class proj28_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj28.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj28.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj28.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj28._transmission
    def _set_transmission(self, bool l):
        proj28._transmission = l

    # Update flag
    def _get_update(self):
        return proj28._update
    def _set_update(self, bool l):
        proj28._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj28._plasticity
    def _set_plasticity(self, bool l):
        proj28._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj28._update_period
    def _set_update_period(self, int l):
        proj28._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj28._update_offset
    def _set_update_offset(self, long l):
        proj28._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj28.get_post_rank()
    def pre_rank_all(self):
        return proj28.get_pre_ranks()
    def pre_rank(self, int n):
        return proj28.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj28.nb_dendrites()
    def nb_synapses(self):
        return proj28.nb_synapses()
    def dendrite_size(self, int n):
        return proj28.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj28.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj28.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj28.size_in_bytes()

    def clear(self):
        return proj28.clear()

# Wrapper for projection 29
@cython.auto_pickle(True)
cdef class proj29_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj29.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj29.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj29.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj29._transmission
    def _set_transmission(self, bool l):
        proj29._transmission = l

    # Update flag
    def _get_update(self):
        return proj29._update
    def _set_update(self, bool l):
        proj29._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj29._plasticity
    def _set_plasticity(self, bool l):
        proj29._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj29._update_period
    def _set_update_period(self, int l):
        proj29._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj29._update_offset
    def _set_update_offset(self, long l):
        proj29._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj29.get_post_rank()
    def pre_rank_all(self):
        return proj29.get_pre_ranks()
    def pre_rank(self, int n):
        return proj29.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj29.nb_dendrites()
    def nb_synapses(self):
        return proj29.nb_synapses()
    def dendrite_size(self, int n):
        return proj29.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj29.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj29.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj29.size_in_bytes()

    def clear(self):
        return proj29.clear()

# Wrapper for projection 30
@cython.auto_pickle(True)
cdef class proj30_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj30.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj30.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj30.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj30._transmission
    def _set_transmission(self, bool l):
        proj30._transmission = l

    # Update flag
    def _get_update(self):
        return proj30._update
    def _set_update(self, bool l):
        proj30._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj30._plasticity
    def _set_plasticity(self, bool l):
        proj30._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj30._update_period
    def _set_update_period(self, int l):
        proj30._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj30._update_offset
    def _set_update_offset(self, long l):
        proj30._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj30.get_post_rank()
    def pre_rank_all(self):
        return proj30.get_pre_ranks()
    def pre_rank(self, int n):
        return proj30.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj30.nb_dendrites()
    def nb_synapses(self):
        return proj30.nb_synapses()
    def dendrite_size(self, int n):
        return proj30.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj30.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj30.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj30.size_in_bytes()

    def clear(self):
        return proj30.clear()

# Wrapper for projection 31
@cython.auto_pickle(True)
cdef class proj31_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj31.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj31.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj31.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj31._transmission
    def _set_transmission(self, bool l):
        proj31._transmission = l

    # Update flag
    def _get_update(self):
        return proj31._update
    def _set_update(self, bool l):
        proj31._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj31._plasticity
    def _set_plasticity(self, bool l):
        proj31._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj31._update_period
    def _set_update_period(self, int l):
        proj31._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj31._update_offset
    def _set_update_offset(self, long l):
        proj31._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj31.get_post_rank()
    def pre_rank_all(self):
        return proj31.get_pre_ranks()
    def pre_rank(self, int n):
        return proj31.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj31.nb_dendrites()
    def nb_synapses(self):
        return proj31.nb_synapses()
    def dendrite_size(self, int n):
        return proj31.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj31.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj31.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj31.size_in_bytes()

    def clear(self):
        return proj31.clear()

# Wrapper for projection 32
@cython.auto_pickle(True)
cdef class proj32_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj32.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj32.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj32.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj32._transmission
    def _set_transmission(self, bool l):
        proj32._transmission = l

    # Update flag
    def _get_update(self):
        return proj32._update
    def _set_update(self, bool l):
        proj32._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj32._plasticity
    def _set_plasticity(self, bool l):
        proj32._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj32._update_period
    def _set_update_period(self, int l):
        proj32._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj32._update_offset
    def _set_update_offset(self, long l):
        proj32._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj32.get_post_rank()
    def pre_rank_all(self):
        return proj32.get_pre_ranks()
    def pre_rank(self, int n):
        return proj32.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj32.nb_dendrites()
    def nb_synapses(self):
        return proj32.nb_synapses()
    def dendrite_size(self, int n):
        return proj32.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj32.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj32.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj32.size_in_bytes()

    def clear(self):
        return proj32.clear()

# Wrapper for projection 33
@cython.auto_pickle(True)
cdef class proj33_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj33.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj33.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj33.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj33._transmission
    def _set_transmission(self, bool l):
        proj33._transmission = l

    # Update flag
    def _get_update(self):
        return proj33._update
    def _set_update(self, bool l):
        proj33._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj33._plasticity
    def _set_plasticity(self, bool l):
        proj33._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj33._update_period
    def _set_update_period(self, int l):
        proj33._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj33._update_offset
    def _set_update_offset(self, long l):
        proj33._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj33.get_post_rank()
    def pre_rank_all(self):
        return proj33.get_pre_ranks()
    def pre_rank(self, int n):
        return proj33.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj33.nb_dendrites()
    def nb_synapses(self):
        return proj33.nb_synapses()
    def dendrite_size(self, int n):
        return proj33.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj33.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj33.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj33.size_in_bytes()

    def clear(self):
        return proj33.clear()

# Wrapper for projection 34
@cython.auto_pickle(True)
cdef class proj34_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj34.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj34.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj34.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj34._transmission
    def _set_transmission(self, bool l):
        proj34._transmission = l

    # Update flag
    def _get_update(self):
        return proj34._update
    def _set_update(self, bool l):
        proj34._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj34._plasticity
    def _set_plasticity(self, bool l):
        proj34._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj34._update_period
    def _set_update_period(self, int l):
        proj34._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj34._update_offset
    def _set_update_offset(self, long l):
        proj34._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj34.get_post_rank()
    def pre_rank_all(self):
        return proj34.get_pre_ranks()
    def pre_rank(self, int n):
        return proj34.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj34.nb_dendrites()
    def nb_synapses(self):
        return proj34.nb_synapses()
    def dendrite_size(self, int n):
        return proj34.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj34.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj34.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj34.size_in_bytes()

    def clear(self):
        return proj34.clear()

# Wrapper for projection 35
@cython.auto_pickle(True)
cdef class proj35_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj35.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj35.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj35.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj35._transmission
    def _set_transmission(self, bool l):
        proj35._transmission = l

    # Update flag
    def _get_update(self):
        return proj35._update
    def _set_update(self, bool l):
        proj35._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj35._plasticity
    def _set_plasticity(self, bool l):
        proj35._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj35._update_period
    def _set_update_period(self, int l):
        proj35._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj35._update_offset
    def _set_update_offset(self, long l):
        proj35._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj35.get_post_rank()
    def pre_rank_all(self):
        return proj35.get_pre_ranks()
    def pre_rank(self, int n):
        return proj35.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj35.nb_dendrites()
    def nb_synapses(self):
        return proj35.nb_synapses()
    def dendrite_size(self, int n):
        return proj35.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj35.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj35.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj35.size_in_bytes()

    def clear(self):
        return proj35.clear()

# Wrapper for projection 36
@cython.auto_pickle(True)
cdef class proj36_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj36.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj36.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj36.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj36._transmission
    def _set_transmission(self, bool l):
        proj36._transmission = l

    # Update flag
    def _get_update(self):
        return proj36._update
    def _set_update(self, bool l):
        proj36._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj36._plasticity
    def _set_plasticity(self, bool l):
        proj36._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj36._update_period
    def _set_update_period(self, int l):
        proj36._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj36._update_offset
    def _set_update_offset(self, long l):
        proj36._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj36.get_post_rank()
    def pre_rank_all(self):
        return proj36.get_pre_ranks()
    def pre_rank(self, int n):
        return proj36.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj36.nb_dendrites()
    def nb_synapses(self):
        return proj36.nb_synapses()
    def dendrite_size(self, int n):
        return proj36.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj36.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj36.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj36.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj36.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj36.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj36.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj36.size_in_bytes()

    def clear(self):
        return proj36.clear()

# Wrapper for projection 37
@cython.auto_pickle(True)
cdef class proj37_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj37.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj37.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj37.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj37._transmission
    def _set_transmission(self, bool l):
        proj37._transmission = l

    # Update flag
    def _get_update(self):
        return proj37._update
    def _set_update(self, bool l):
        proj37._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj37._plasticity
    def _set_plasticity(self, bool l):
        proj37._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj37._update_period
    def _set_update_period(self, int l):
        proj37._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj37._update_offset
    def _set_update_offset(self, long l):
        proj37._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj37.get_post_rank()
    def pre_rank_all(self):
        return proj37.get_pre_ranks()
    def pre_rank(self, int n):
        return proj37.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj37.nb_dendrites()
    def nb_synapses(self):
        return proj37.nb_synapses()
    def dendrite_size(self, int n):
        return proj37.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj37.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj37.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj37.size_in_bytes()

    def clear(self):
        return proj37.clear()

# Wrapper for projection 38
@cython.auto_pickle(True)
cdef class proj38_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj38.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj38.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj38.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj38._transmission
    def _set_transmission(self, bool l):
        proj38._transmission = l

    # Update flag
    def _get_update(self):
        return proj38._update
    def _set_update(self, bool l):
        proj38._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj38._plasticity
    def _set_plasticity(self, bool l):
        proj38._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj38._update_period
    def _set_update_period(self, int l):
        proj38._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj38._update_offset
    def _set_update_offset(self, long l):
        proj38._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj38.get_post_rank()
    def pre_rank_all(self):
        return proj38.get_pre_ranks()
    def pre_rank(self, int n):
        return proj38.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj38.nb_dendrites()
    def nb_synapses(self):
        return proj38.nb_synapses()
    def dendrite_size(self, int n):
        return proj38.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj38.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj38.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj38.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj38.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj38.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj38.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj38.size_in_bytes()

    def clear(self):
        return proj38.clear()

# Wrapper for projection 39
@cython.auto_pickle(True)
cdef class proj39_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj39.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj39.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj39.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj39._transmission
    def _set_transmission(self, bool l):
        proj39._transmission = l

    # Update flag
    def _get_update(self):
        return proj39._update
    def _set_update(self, bool l):
        proj39._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj39._plasticity
    def _set_plasticity(self, bool l):
        proj39._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj39._update_period
    def _set_update_period(self, int l):
        proj39._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj39._update_offset
    def _set_update_offset(self, long l):
        proj39._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj39.get_post_rank()
    def pre_rank_all(self):
        return proj39.get_pre_ranks()
    def pre_rank(self, int n):
        return proj39.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj39.nb_dendrites()
    def nb_synapses(self):
        return proj39.nb_synapses()
    def dendrite_size(self, int n):
        return proj39.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj39.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj39.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj39.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj39.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj39.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj39.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)


    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj39.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj39.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj39.size_in_bytes()

    def clear(self):
        return proj39.clear()

# Wrapper for projection 40
@cython.auto_pickle(True)
cdef class proj40_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj40.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj40.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj40.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj40._transmission
    def _set_transmission(self, bool l):
        proj40._transmission = l

    # Update flag
    def _get_update(self):
        return proj40._update
    def _set_update(self, bool l):
        proj40._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj40._plasticity
    def _set_plasticity(self, bool l):
        proj40._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj40._update_period
    def _set_update_period(self, int l):
        proj40._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj40._update_offset
    def _set_update_offset(self, long l):
        proj40._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj40.get_post_rank()
    def pre_rank_all(self):
        return proj40.get_pre_ranks()
    def pre_rank(self, int n):
        return proj40.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj40.nb_dendrites()
    def nb_synapses(self):
        return proj40.nb_synapses()
    def dendrite_size(self, int n):
        return proj40.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj40.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj40.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj40.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj40.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj40.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj40.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)


    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj40.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj40.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj40.size_in_bytes()

    def clear(self):
        return proj40.clear()

# Wrapper for projection 41
@cython.auto_pickle(True)
cdef class proj41_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj41.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj41.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj41.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj41._transmission
    def _set_transmission(self, bool l):
        proj41._transmission = l

    # Update flag
    def _get_update(self):
        return proj41._update
    def _set_update(self, bool l):
        proj41._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj41._plasticity
    def _set_plasticity(self, bool l):
        proj41._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj41._update_period
    def _set_update_period(self, int l):
        proj41._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj41._update_offset
    def _set_update_offset(self, long l):
        proj41._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj41.get_post_rank()
    def pre_rank_all(self):
        return proj41.get_pre_ranks()
    def pre_rank(self, int n):
        return proj41.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj41.nb_dendrites()
    def nb_synapses(self):
        return proj41.nb_synapses()
    def dendrite_size(self, int n):
        return proj41.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj41.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj41.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj41.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj41.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj41.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj41.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)


    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj41.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj41.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj41.size_in_bytes()

    def clear(self):
        return proj41.clear()

# Wrapper for projection 42
@cython.auto_pickle(True)
cdef class proj42_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj42.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj42.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj42.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj42._transmission
    def _set_transmission(self, bool l):
        proj42._transmission = l

    # Update flag
    def _get_update(self):
        return proj42._update
    def _set_update(self, bool l):
        proj42._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj42._plasticity
    def _set_plasticity(self, bool l):
        proj42._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj42._update_period
    def _set_update_period(self, int l):
        proj42._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj42._update_offset
    def _set_update_offset(self, long l):
        proj42._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj42.get_post_rank()
    def pre_rank_all(self):
        return proj42.get_pre_ranks()
    def pre_rank(self, int n):
        return proj42.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj42.nb_dendrites()
    def nb_synapses(self):
        return proj42.nb_synapses()
    def dendrite_size(self, int n):
        return proj42.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj42.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj42.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj42.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj42.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj42.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj42.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)


    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj42.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj42.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj42.size_in_bytes()

    def clear(self):
        return proj42.clear()

# Wrapper for projection 43
@cython.auto_pickle(True)
cdef class proj43_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj43.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj43.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj43.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj43._transmission
    def _set_transmission(self, bool l):
        proj43._transmission = l

    # Update flag
    def _get_update(self):
        return proj43._update
    def _set_update(self, bool l):
        proj43._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj43._plasticity
    def _set_plasticity(self, bool l):
        proj43._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj43._update_period
    def _set_update_period(self, int l):
        proj43._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj43._update_offset
    def _set_update_offset(self, long l):
        proj43._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj43.get_post_rank()
    def pre_rank_all(self):
        return proj43.get_pre_ranks()
    def pre_rank(self, int n):
        return proj43.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj43.nb_dendrites()
    def nb_synapses(self):
        return proj43.nb_synapses()
    def dendrite_size(self, int n):
        return proj43.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj43.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj43.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj43.size_in_bytes()

    def clear(self):
        return proj43.clear()

# Wrapper for projection 44
@cython.auto_pickle(True)
cdef class proj44_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj44.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj44.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj44.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj44._transmission
    def _set_transmission(self, bool l):
        proj44._transmission = l

    # Update flag
    def _get_update(self):
        return proj44._update
    def _set_update(self, bool l):
        proj44._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj44._plasticity
    def _set_plasticity(self, bool l):
        proj44._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj44._update_period
    def _set_update_period(self, int l):
        proj44._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj44._update_offset
    def _set_update_offset(self, long l):
        proj44._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj44.get_post_rank()
    def pre_rank_all(self):
        return proj44.get_pre_ranks()
    def pre_rank(self, int n):
        return proj44.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj44.nb_dendrites()
    def nb_synapses(self):
        return proj44.nb_synapses()
    def dendrite_size(self, int n):
        return proj44.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj44.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj44.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj44.size_in_bytes()

    def clear(self):
        return proj44.clear()

# Wrapper for projection 45
@cython.auto_pickle(True)
cdef class proj45_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj45.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj45.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj45.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj45._transmission
    def _set_transmission(self, bool l):
        proj45._transmission = l

    # Update flag
    def _get_update(self):
        return proj45._update
    def _set_update(self, bool l):
        proj45._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj45._plasticity
    def _set_plasticity(self, bool l):
        proj45._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj45._update_period
    def _set_update_period(self, int l):
        proj45._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj45._update_offset
    def _set_update_offset(self, long l):
        proj45._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj45.get_post_rank()
    def pre_rank_all(self):
        return proj45.get_pre_ranks()
    def pre_rank(self, int n):
        return proj45.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj45.nb_dendrites()
    def nb_synapses(self):
        return proj45.nb_synapses()
    def dendrite_size(self, int n):
        return proj45.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj45.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj45.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj45.size_in_bytes()

    def clear(self):
        return proj45.clear()

# Wrapper for projection 46
@cython.auto_pickle(True)
cdef class proj46_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj46.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj46.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj46.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj46._transmission
    def _set_transmission(self, bool l):
        proj46._transmission = l

    # Update flag
    def _get_update(self):
        return proj46._update
    def _set_update(self, bool l):
        proj46._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj46._plasticity
    def _set_plasticity(self, bool l):
        proj46._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj46._update_period
    def _set_update_period(self, int l):
        proj46._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj46._update_offset
    def _set_update_offset(self, long l):
        proj46._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj46.get_post_rank()
    def pre_rank_all(self):
        return proj46.get_pre_ranks()
    def pre_rank(self, int n):
        return proj46.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj46.nb_dendrites()
    def nb_synapses(self):
        return proj46.nb_synapses()
    def dendrite_size(self, int n):
        return proj46.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj46.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj46.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj46.size_in_bytes()

    def clear(self):
        return proj46.clear()

# Wrapper for projection 47
@cython.auto_pickle(True)
cdef class proj47_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj47.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj47.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj47.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj47._transmission
    def _set_transmission(self, bool l):
        proj47._transmission = l

    # Update flag
    def _get_update(self):
        return proj47._update
    def _set_update(self, bool l):
        proj47._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj47._plasticity
    def _set_plasticity(self, bool l):
        proj47._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj47._update_period
    def _set_update_period(self, int l):
        proj47._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj47._update_offset
    def _set_update_offset(self, long l):
        proj47._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj47.get_post_rank()
    def pre_rank_all(self):
        return proj47.get_pre_ranks()
    def pre_rank(self, int n):
        return proj47.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj47.nb_dendrites()
    def nb_synapses(self):
        return proj47.nb_synapses()
    def dendrite_size(self, int n):
        return proj47.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj47.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj47.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj47.size_in_bytes()

    def clear(self):
        return proj47.clear()

# Wrapper for projection 48
@cython.auto_pickle(True)
cdef class proj48_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj48.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj48.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj48.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj48._transmission
    def _set_transmission(self, bool l):
        proj48._transmission = l

    # Update flag
    def _get_update(self):
        return proj48._update
    def _set_update(self, bool l):
        proj48._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj48._plasticity
    def _set_plasticity(self, bool l):
        proj48._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj48._update_period
    def _set_update_period(self, int l):
        proj48._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj48._update_offset
    def _set_update_offset(self, long l):
        proj48._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj48.get_post_rank()
    def pre_rank_all(self):
        return proj48.get_pre_ranks()
    def pre_rank(self, int n):
        return proj48.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj48.nb_dendrites()
    def nb_synapses(self):
        return proj48.nb_synapses()
    def dendrite_size(self, int n):
        return proj48.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj48.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj48.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj48.size_in_bytes()

    def clear(self):
        return proj48.clear()

# Wrapper for projection 49
@cython.auto_pickle(True)
cdef class proj49_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj49.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj49.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj49.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj49._transmission
    def _set_transmission(self, bool l):
        proj49._transmission = l

    # Update flag
    def _get_update(self):
        return proj49._update
    def _set_update(self, bool l):
        proj49._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj49._plasticity
    def _set_plasticity(self, bool l):
        proj49._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj49._update_period
    def _set_update_period(self, int l):
        proj49._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj49._update_offset
    def _set_update_offset(self, long l):
        proj49._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj49.get_post_rank()
    def pre_rank_all(self):
        return proj49.get_pre_ranks()
    def pre_rank(self, int n):
        return proj49.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj49.nb_dendrites()
    def nb_synapses(self):
        return proj49.nb_synapses()
    def dendrite_size(self, int n):
        return proj49.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj49.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj49.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj49.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj49.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj49.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj49.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj49.size_in_bytes()

    def clear(self):
        return proj49.clear()

# Wrapper for projection 50
@cython.auto_pickle(True)
cdef class proj50_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj50.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj50.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj50.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj50._transmission
    def _set_transmission(self, bool l):
        proj50._transmission = l

    # Update flag
    def _get_update(self):
        return proj50._update
    def _set_update(self, bool l):
        proj50._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj50._plasticity
    def _set_plasticity(self, bool l):
        proj50._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj50._update_period
    def _set_update_period(self, int l):
        proj50._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj50._update_offset
    def _set_update_offset(self, long l):
        proj50._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj50.get_post_rank()
    def pre_rank_all(self):
        return proj50.get_pre_ranks()
    def pre_rank(self, int n):
        return proj50.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj50.nb_dendrites()
    def nb_synapses(self):
        return proj50.nb_synapses()
    def dendrite_size(self, int n):
        return proj50.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj50.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj50.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj50.size_in_bytes()

    def clear(self):
        return proj50.clear()

# Wrapper for projection 51
@cython.auto_pickle(True)
cdef class proj51_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj51.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj51.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj51.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj51._transmission
    def _set_transmission(self, bool l):
        proj51._transmission = l

    # Update flag
    def _get_update(self):
        return proj51._update
    def _set_update(self, bool l):
        proj51._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj51._plasticity
    def _set_plasticity(self, bool l):
        proj51._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj51._update_period
    def _set_update_period(self, int l):
        proj51._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj51._update_offset
    def _set_update_offset(self, long l):
        proj51._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj51.get_post_rank()
    def pre_rank_all(self):
        return proj51.get_pre_ranks()
    def pre_rank(self, int n):
        return proj51.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj51.nb_dendrites()
    def nb_synapses(self):
        return proj51.nb_synapses()
    def dendrite_size(self, int n):
        return proj51.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj51.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj51.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj51.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj51.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj51.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj51.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj51.size_in_bytes()

    def clear(self):
        return proj51.clear()

# Wrapper for projection 52
@cython.auto_pickle(True)
cdef class proj52_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj52.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj52.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj52.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj52._transmission
    def _set_transmission(self, bool l):
        proj52._transmission = l

    # Update flag
    def _get_update(self):
        return proj52._update
    def _set_update(self, bool l):
        proj52._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj52._plasticity
    def _set_plasticity(self, bool l):
        proj52._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj52._update_period
    def _set_update_period(self, int l):
        proj52._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj52._update_offset
    def _set_update_offset(self, long l):
        proj52._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj52.get_post_rank()
    def pre_rank_all(self):
        return proj52.get_pre_ranks()
    def pre_rank(self, int n):
        return proj52.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj52.nb_dendrites()
    def nb_synapses(self):
        return proj52.nb_synapses()
    def dendrite_size(self, int n):
        return proj52.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj52.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj52.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj52.size_in_bytes()

    def clear(self):
        return proj52.clear()

# Wrapper for projection 53
@cython.auto_pickle(True)
cdef class proj53_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj53.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj53.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj53.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj53._transmission
    def _set_transmission(self, bool l):
        proj53._transmission = l

    # Update flag
    def _get_update(self):
        return proj53._update
    def _set_update(self, bool l):
        proj53._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj53._plasticity
    def _set_plasticity(self, bool l):
        proj53._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj53._update_period
    def _set_update_period(self, int l):
        proj53._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj53._update_offset
    def _set_update_offset(self, long l):
        proj53._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj53.get_post_rank()
    def pre_rank_all(self):
        return proj53.get_pre_ranks()
    def pre_rank(self, int n):
        return proj53.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj53.nb_dendrites()
    def nb_synapses(self):
        return proj53.nb_synapses()
    def dendrite_size(self, int n):
        return proj53.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj53.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj53.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj53.size_in_bytes()

    def clear(self):
        return proj53.clear()

# Wrapper for projection 54
@cython.auto_pickle(True)
cdef class proj54_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj54.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj54.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj54.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj54._transmission
    def _set_transmission(self, bool l):
        proj54._transmission = l

    # Update flag
    def _get_update(self):
        return proj54._update
    def _set_update(self, bool l):
        proj54._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj54._plasticity
    def _set_plasticity(self, bool l):
        proj54._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj54._update_period
    def _set_update_period(self, int l):
        proj54._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj54._update_offset
    def _set_update_offset(self, long l):
        proj54._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj54.get_post_rank()
    def pre_rank_all(self):
        return proj54.get_pre_ranks()
    def pre_rank(self, int n):
        return proj54.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj54.nb_dendrites()
    def nb_synapses(self):
        return proj54.nb_synapses()
    def dendrite_size(self, int n):
        return proj54.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj54.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj54.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj54.size_in_bytes()

    def clear(self):
        return proj54.clear()

# Wrapper for projection 55
@cython.auto_pickle(True)
cdef class proj55_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj55.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj55.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj55.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj55._transmission
    def _set_transmission(self, bool l):
        proj55._transmission = l

    # Update flag
    def _get_update(self):
        return proj55._update
    def _set_update(self, bool l):
        proj55._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj55._plasticity
    def _set_plasticity(self, bool l):
        proj55._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj55._update_period
    def _set_update_period(self, int l):
        proj55._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj55._update_offset
    def _set_update_offset(self, long l):
        proj55._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj55.get_post_rank()
    def pre_rank_all(self):
        return proj55.get_pre_ranks()
    def pre_rank(self, int n):
        return proj55.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj55.nb_dendrites()
    def nb_synapses(self):
        return proj55.nb_synapses()
    def dendrite_size(self, int n):
        return proj55.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj55.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj55.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj55.size_in_bytes()

    def clear(self):
        return proj55.clear()

# Wrapper for projection 56
@cython.auto_pickle(True)
cdef class proj56_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj56.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj56.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj56.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj56._transmission
    def _set_transmission(self, bool l):
        proj56._transmission = l

    # Update flag
    def _get_update(self):
        return proj56._update
    def _set_update(self, bool l):
        proj56._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj56._plasticity
    def _set_plasticity(self, bool l):
        proj56._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj56._update_period
    def _set_update_period(self, int l):
        proj56._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj56._update_offset
    def _set_update_offset(self, long l):
        proj56._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj56.get_post_rank()
    def pre_rank_all(self):
        return proj56.get_pre_ranks()
    def pre_rank(self, int n):
        return proj56.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj56.nb_dendrites()
    def nb_synapses(self):
        return proj56.nb_synapses()
    def dendrite_size(self, int n):
        return proj56.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj56.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj56.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj56.size_in_bytes()

    def clear(self):
        return proj56.clear()

# Wrapper for projection 57
@cython.auto_pickle(True)
cdef class proj57_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj57.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj57.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj57.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj57._transmission
    def _set_transmission(self, bool l):
        proj57._transmission = l

    # Update flag
    def _get_update(self):
        return proj57._update
    def _set_update(self, bool l):
        proj57._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj57._plasticity
    def _set_plasticity(self, bool l):
        proj57._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj57._update_period
    def _set_update_period(self, int l):
        proj57._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj57._update_offset
    def _set_update_offset(self, long l):
        proj57._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj57.get_post_rank()
    def pre_rank_all(self):
        return proj57.get_pre_ranks()
    def pre_rank(self, int n):
        return proj57.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj57.nb_dendrites()
    def nb_synapses(self):
        return proj57.nb_synapses()
    def dendrite_size(self, int n):
        return proj57.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj57.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj57.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj57.size_in_bytes()

    def clear(self):
        return proj57.clear()

# Wrapper for projection 58
@cython.auto_pickle(True)
cdef class proj58_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj58.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj58.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj58.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj58._transmission
    def _set_transmission(self, bool l):
        proj58._transmission = l

    # Update flag
    def _get_update(self):
        return proj58._update
    def _set_update(self, bool l):
        proj58._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj58._plasticity
    def _set_plasticity(self, bool l):
        proj58._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj58._update_period
    def _set_update_period(self, int l):
        proj58._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj58._update_offset
    def _set_update_offset(self, long l):
        proj58._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj58.get_post_rank()
    def pre_rank_all(self):
        return proj58.get_pre_ranks()
    def pre_rank(self, int n):
        return proj58.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj58.nb_dendrites()
    def nb_synapses(self):
        return proj58.nb_synapses()
    def dendrite_size(self, int n):
        return proj58.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj58.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj58.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj58.size_in_bytes()

    def clear(self):
        return proj58.clear()

# Wrapper for projection 59
@cython.auto_pickle(True)
cdef class proj59_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj59.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj59.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj59.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj59._transmission
    def _set_transmission(self, bool l):
        proj59._transmission = l

    # Update flag
    def _get_update(self):
        return proj59._update
    def _set_update(self, bool l):
        proj59._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj59._plasticity
    def _set_plasticity(self, bool l):
        proj59._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj59._update_period
    def _set_update_period(self, int l):
        proj59._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj59._update_offset
    def _set_update_offset(self, long l):
        proj59._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj59.get_post_rank()
    def pre_rank_all(self):
        return proj59.get_pre_ranks()
    def pre_rank(self, int n):
        return proj59.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj59.nb_dendrites()
    def nb_synapses(self):
        return proj59.nb_synapses()
    def dendrite_size(self, int n):
        return proj59.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj59.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj59.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj59.size_in_bytes()

    def clear(self):
        return proj59.clear()

# Wrapper for projection 60
@cython.auto_pickle(True)
cdef class proj60_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj60.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj60.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj60.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj60._transmission
    def _set_transmission(self, bool l):
        proj60._transmission = l

    # Update flag
    def _get_update(self):
        return proj60._update
    def _set_update(self, bool l):
        proj60._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj60._plasticity
    def _set_plasticity(self, bool l):
        proj60._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj60._update_period
    def _set_update_period(self, int l):
        proj60._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj60._update_offset
    def _set_update_offset(self, long l):
        proj60._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj60.get_post_rank()
    def pre_rank_all(self):
        return proj60.get_pre_ranks()
    def pre_rank(self, int n):
        return proj60.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj60.nb_dendrites()
    def nb_synapses(self):
        return proj60.nb_synapses()
    def dendrite_size(self, int n):
        return proj60.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj60.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj60.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj60.size_in_bytes()

    def clear(self):
        return proj60.clear()

# Wrapper for projection 61
@cython.auto_pickle(True)
cdef class proj61_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj61.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj61.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj61.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj61._transmission
    def _set_transmission(self, bool l):
        proj61._transmission = l

    # Update flag
    def _get_update(self):
        return proj61._update
    def _set_update(self, bool l):
        proj61._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj61._plasticity
    def _set_plasticity(self, bool l):
        proj61._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj61._update_period
    def _set_update_period(self, int l):
        proj61._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj61._update_offset
    def _set_update_offset(self, long l):
        proj61._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj61.get_post_rank()
    def pre_rank_all(self):
        return proj61.get_pre_ranks()
    def pre_rank(self, int n):
        return proj61.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj61.nb_dendrites()
    def nb_synapses(self):
        return proj61.nb_synapses()
    def dendrite_size(self, int n):
        return proj61.dendrite_size(n)




    # Local Attribute
    def get_local_attribute_all(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj61.get_local_attribute_all_double(cpp_string)


    def get_local_attribute_row(self, name, rk_post, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj61.get_local_attribute_row_double(cpp_string, rk_post)


    def get_local_attribute(self, name, rk_post, rk_pre, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            return proj61.get_local_attribute_double(cpp_string, rk_post, rk_pre)


    def set_local_attribute_all(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj61.set_local_attribute_all_double(cpp_string, value)


    def set_local_attribute_row(self, name, rk_post, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj61.set_local_attribute_row_double(cpp_string, rk_post, value)


    def set_local_attribute(self, name, rk_post, rk_pre, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":
            proj61.set_local_attribute_double(cpp_string, rk_post, rk_pre, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj61.size_in_bytes()

    def clear(self):
        return proj61.clear()

# Wrapper for projection 62
@cython.auto_pickle(True)
cdef class proj62_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj62.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj62.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj62.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj62._transmission
    def _set_transmission(self, bool l):
        proj62._transmission = l

    # Update flag
    def _get_update(self):
        return proj62._update
    def _set_update(self, bool l):
        proj62._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj62._plasticity
    def _set_plasticity(self, bool l):
        proj62._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj62._update_period
    def _set_update_period(self, int l):
        proj62._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj62._update_offset
    def _set_update_offset(self, long l):
        proj62._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj62.get_post_rank()
    def pre_rank_all(self):
        return proj62.get_pre_ranks()
    def pre_rank(self, int n):
        return proj62.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj62.nb_dendrites()
    def nb_synapses(self):
        return proj62.nb_synapses()
    def dendrite_size(self, int n):
        return proj62.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj62.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj62.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj62.size_in_bytes()

    def clear(self):
        return proj62.clear()

# Wrapper for projection 63
@cython.auto_pickle(True)
cdef class proj63_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj63.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj63.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj63.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj63._transmission
    def _set_transmission(self, bool l):
        proj63._transmission = l

    # Update flag
    def _get_update(self):
        return proj63._update
    def _set_update(self, bool l):
        proj63._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj63._plasticity
    def _set_plasticity(self, bool l):
        proj63._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj63._update_period
    def _set_update_period(self, int l):
        proj63._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj63._update_offset
    def _set_update_offset(self, long l):
        proj63._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj63.get_post_rank()
    def pre_rank_all(self):
        return proj63.get_pre_ranks()
    def pre_rank(self, int n):
        return proj63.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj63.nb_dendrites()
    def nb_synapses(self):
        return proj63.nb_synapses()
    def dendrite_size(self, int n):
        return proj63.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj63.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj63.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj63.size_in_bytes()

    def clear(self):
        return proj63.clear()

# Wrapper for projection 64
@cython.auto_pickle(True)
cdef class proj64_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj64.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj64.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj64.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj64._transmission
    def _set_transmission(self, bool l):
        proj64._transmission = l

    # Update flag
    def _get_update(self):
        return proj64._update
    def _set_update(self, bool l):
        proj64._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj64._plasticity
    def _set_plasticity(self, bool l):
        proj64._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj64._update_period
    def _set_update_period(self, int l):
        proj64._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj64._update_offset
    def _set_update_offset(self, long l):
        proj64._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj64.get_post_rank()
    def pre_rank_all(self):
        return proj64.get_pre_ranks()
    def pre_rank(self, int n):
        return proj64.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj64.nb_dendrites()
    def nb_synapses(self):
        return proj64.nb_synapses()
    def dendrite_size(self, int n):
        return proj64.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj64.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj64.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj64.size_in_bytes()

    def clear(self):
        return proj64.clear()

# Wrapper for projection 65
@cython.auto_pickle(True)
cdef class proj65_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj65.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj65.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj65.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj65._transmission
    def _set_transmission(self, bool l):
        proj65._transmission = l

    # Update flag
    def _get_update(self):
        return proj65._update
    def _set_update(self, bool l):
        proj65._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj65._plasticity
    def _set_plasticity(self, bool l):
        proj65._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj65._update_period
    def _set_update_period(self, int l):
        proj65._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj65._update_offset
    def _set_update_offset(self, long l):
        proj65._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj65.get_post_rank()
    def pre_rank_all(self):
        return proj65.get_pre_ranks()
    def pre_rank(self, int n):
        return proj65.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj65.nb_dendrites()
    def nb_synapses(self):
        return proj65.nb_synapses()
    def dendrite_size(self, int n):
        return proj65.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj65.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj65.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj65.size_in_bytes()

    def clear(self):
        return proj65.clear()

# Wrapper for projection 66
@cython.auto_pickle(True)
cdef class proj66_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj66.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj66.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj66.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj66._transmission
    def _set_transmission(self, bool l):
        proj66._transmission = l

    # Update flag
    def _get_update(self):
        return proj66._update
    def _set_update(self, bool l):
        proj66._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj66._plasticity
    def _set_plasticity(self, bool l):
        proj66._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj66._update_period
    def _set_update_period(self, int l):
        proj66._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj66._update_offset
    def _set_update_offset(self, long l):
        proj66._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj66.get_post_rank()
    def pre_rank_all(self):
        return proj66.get_pre_ranks()
    def pre_rank(self, int n):
        return proj66.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj66.nb_dendrites()
    def nb_synapses(self):
        return proj66.nb_synapses()
    def dendrite_size(self, int n):
        return proj66.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj66.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj66.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj66.size_in_bytes()

    def clear(self):
        return proj66.clear()

# Wrapper for projection 67
@cython.auto_pickle(True)
cdef class proj67_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj67.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj67.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj67.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj67._transmission
    def _set_transmission(self, bool l):
        proj67._transmission = l

    # Update flag
    def _get_update(self):
        return proj67._update
    def _set_update(self, bool l):
        proj67._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj67._plasticity
    def _set_plasticity(self, bool l):
        proj67._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj67._update_period
    def _set_update_period(self, int l):
        proj67._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj67._update_offset
    def _set_update_offset(self, long l):
        proj67._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj67.get_post_rank()
    def pre_rank_all(self):
        return proj67.get_pre_ranks()
    def pre_rank(self, int n):
        return proj67.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj67.nb_dendrites()
    def nb_synapses(self):
        return proj67.nb_synapses()
    def dendrite_size(self, int n):
        return proj67.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj67.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj67.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj67.size_in_bytes()

    def clear(self):
        return proj67.clear()

# Wrapper for projection 68
@cython.auto_pickle(True)
cdef class proj68_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj68.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj68.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj68.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj68._transmission
    def _set_transmission(self, bool l):
        proj68._transmission = l

    # Update flag
    def _get_update(self):
        return proj68._update
    def _set_update(self, bool l):
        proj68._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj68._plasticity
    def _set_plasticity(self, bool l):
        proj68._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj68._update_period
    def _set_update_period(self, int l):
        proj68._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj68._update_offset
    def _set_update_offset(self, long l):
        proj68._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj68.get_post_rank()
    def pre_rank_all(self):
        return proj68.get_pre_ranks()
    def pre_rank(self, int n):
        return proj68.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj68.nb_dendrites()
    def nb_synapses(self):
        return proj68.nb_synapses()
    def dendrite_size(self, int n):
        return proj68.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj68.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj68.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj68.size_in_bytes()

    def clear(self):
        return proj68.clear()

# Wrapper for projection 69
@cython.auto_pickle(True)
cdef class proj69_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj69.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj69.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj69.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj69._transmission
    def _set_transmission(self, bool l):
        proj69._transmission = l

    # Update flag
    def _get_update(self):
        return proj69._update
    def _set_update(self, bool l):
        proj69._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj69._plasticity
    def _set_plasticity(self, bool l):
        proj69._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj69._update_period
    def _set_update_period(self, int l):
        proj69._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj69._update_offset
    def _set_update_offset(self, long l):
        proj69._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj69.get_post_rank()
    def pre_rank_all(self):
        return proj69.get_pre_ranks()
    def pre_rank(self, int n):
        return proj69.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj69.nb_dendrites()
    def nb_synapses(self):
        return proj69.nb_synapses()
    def dendrite_size(self, int n):
        return proj69.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj69.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj69.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj69.size_in_bytes()

    def clear(self):
        return proj69.clear()

# Wrapper for projection 70
@cython.auto_pickle(True)
cdef class proj70_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj70.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj70.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj70.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj70._transmission
    def _set_transmission(self, bool l):
        proj70._transmission = l

    # Update flag
    def _get_update(self):
        return proj70._update
    def _set_update(self, bool l):
        proj70._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj70._plasticity
    def _set_plasticity(self, bool l):
        proj70._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj70._update_period
    def _set_update_period(self, int l):
        proj70._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj70._update_offset
    def _set_update_offset(self, long l):
        proj70._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj70.get_post_rank()
    def pre_rank_all(self):
        return proj70.get_pre_ranks()
    def pre_rank(self, int n):
        return proj70.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj70.nb_dendrites()
    def nb_synapses(self):
        return proj70.nb_synapses()
    def dendrite_size(self, int n):
        return proj70.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj70.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj70.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj70.size_in_bytes()

    def clear(self):
        return proj70.clear()

# Wrapper for projection 71
@cython.auto_pickle(True)
cdef class proj71_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj71.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj71.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj71.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj71._transmission
    def _set_transmission(self, bool l):
        proj71._transmission = l

    # Update flag
    def _get_update(self):
        return proj71._update
    def _set_update(self, bool l):
        proj71._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj71._plasticity
    def _set_plasticity(self, bool l):
        proj71._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj71._update_period
    def _set_update_period(self, int l):
        proj71._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj71._update_offset
    def _set_update_offset(self, long l):
        proj71._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj71.get_post_rank()
    def pre_rank_all(self):
        return proj71.get_pre_ranks()
    def pre_rank(self, int n):
        return proj71.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj71.nb_dendrites()
    def nb_synapses(self):
        return proj71.nb_synapses()
    def dendrite_size(self, int n):
        return proj71.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj71.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj71.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj71.size_in_bytes()

    def clear(self):
        return proj71.clear()

# Wrapper for projection 72
@cython.auto_pickle(True)
cdef class proj72_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj72.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj72.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj72.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj72._transmission
    def _set_transmission(self, bool l):
        proj72._transmission = l

    # Update flag
    def _get_update(self):
        return proj72._update
    def _set_update(self, bool l):
        proj72._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj72._plasticity
    def _set_plasticity(self, bool l):
        proj72._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj72._update_period
    def _set_update_period(self, int l):
        proj72._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj72._update_offset
    def _set_update_offset(self, long l):
        proj72._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj72.get_post_rank()
    def pre_rank_all(self):
        return proj72.get_pre_ranks()
    def pre_rank(self, int n):
        return proj72.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj72.nb_dendrites()
    def nb_synapses(self):
        return proj72.nb_synapses()
    def dendrite_size(self, int n):
        return proj72.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj72.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj72.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj72.size_in_bytes()

    def clear(self):
        return proj72.clear()

# Wrapper for projection 73
@cython.auto_pickle(True)
cdef class proj73_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj73.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj73.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj73.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj73._transmission
    def _set_transmission(self, bool l):
        proj73._transmission = l

    # Update flag
    def _get_update(self):
        return proj73._update
    def _set_update(self, bool l):
        proj73._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj73._plasticity
    def _set_plasticity(self, bool l):
        proj73._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj73._update_period
    def _set_update_period(self, int l):
        proj73._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj73._update_offset
    def _set_update_offset(self, long l):
        proj73._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj73.get_post_rank()
    def pre_rank_all(self):
        return proj73.get_pre_ranks()
    def pre_rank(self, int n):
        return proj73.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj73.nb_dendrites()
    def nb_synapses(self):
        return proj73.nb_synapses()
    def dendrite_size(self, int n):
        return proj73.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj73.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj73.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj73.size_in_bytes()

    def clear(self):
        return proj73.clear()

# Wrapper for projection 74
@cython.auto_pickle(True)
cdef class proj74_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj74.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj74.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj74.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj74._transmission
    def _set_transmission(self, bool l):
        proj74._transmission = l

    # Update flag
    def _get_update(self):
        return proj74._update
    def _set_update(self, bool l):
        proj74._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj74._plasticity
    def _set_plasticity(self, bool l):
        proj74._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj74._update_period
    def _set_update_period(self, int l):
        proj74._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj74._update_offset
    def _set_update_offset(self, long l):
        proj74._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj74.get_post_rank()
    def pre_rank_all(self):
        return proj74.get_pre_ranks()
    def pre_rank(self, int n):
        return proj74.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj74.nb_dendrites()
    def nb_synapses(self):
        return proj74.nb_synapses()
    def dendrite_size(self, int n):
        return proj74.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj74.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj74.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj74.size_in_bytes()

    def clear(self):
        return proj74.clear()

# Wrapper for projection 75
@cython.auto_pickle(True)
cdef class proj75_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj75.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj75.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj75.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj75._transmission
    def _set_transmission(self, bool l):
        proj75._transmission = l

    # Update flag
    def _get_update(self):
        return proj75._update
    def _set_update(self, bool l):
        proj75._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj75._plasticity
    def _set_plasticity(self, bool l):
        proj75._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj75._update_period
    def _set_update_period(self, int l):
        proj75._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj75._update_offset
    def _set_update_offset(self, long l):
        proj75._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj75.get_post_rank()
    def pre_rank_all(self):
        return proj75.get_pre_ranks()
    def pre_rank(self, int n):
        return proj75.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj75.nb_dendrites()
    def nb_synapses(self):
        return proj75.nb_synapses()
    def dendrite_size(self, int n):
        return proj75.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj75.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj75.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj75.size_in_bytes()

    def clear(self):
        return proj75.clear()

# Wrapper for projection 76
@cython.auto_pickle(True)
cdef class proj76_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj76.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj76.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj76.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj76._transmission
    def _set_transmission(self, bool l):
        proj76._transmission = l

    # Update flag
    def _get_update(self):
        return proj76._update
    def _set_update(self, bool l):
        proj76._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj76._plasticity
    def _set_plasticity(self, bool l):
        proj76._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj76._update_period
    def _set_update_period(self, int l):
        proj76._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj76._update_offset
    def _set_update_offset(self, long l):
        proj76._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj76.get_post_rank()
    def pre_rank_all(self):
        return proj76.get_pre_ranks()
    def pre_rank(self, int n):
        return proj76.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj76.nb_dendrites()
    def nb_synapses(self):
        return proj76.nb_synapses()
    def dendrite_size(self, int n):
        return proj76.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj76.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj76.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj76.size_in_bytes()

    def clear(self):
        return proj76.clear()

# Wrapper for projection 77
@cython.auto_pickle(True)
cdef class proj77_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj77.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj77.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj77.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj77._transmission
    def _set_transmission(self, bool l):
        proj77._transmission = l

    # Update flag
    def _get_update(self):
        return proj77._update
    def _set_update(self, bool l):
        proj77._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj77._plasticity
    def _set_plasticity(self, bool l):
        proj77._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj77._update_period
    def _set_update_period(self, int l):
        proj77._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj77._update_offset
    def _set_update_offset(self, long l):
        proj77._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj77.get_post_rank()
    def pre_rank_all(self):
        return proj77.get_pre_ranks()
    def pre_rank(self, int n):
        return proj77.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj77.nb_dendrites()
    def nb_synapses(self):
        return proj77.nb_synapses()
    def dendrite_size(self, int n):
        return proj77.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj77.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj77.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj77.size_in_bytes()

    def clear(self):
        return proj77.clear()

# Wrapper for projection 78
@cython.auto_pickle(True)
cdef class proj78_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj78.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj78.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj78.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj78._transmission
    def _set_transmission(self, bool l):
        proj78._transmission = l

    # Update flag
    def _get_update(self):
        return proj78._update
    def _set_update(self, bool l):
        proj78._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj78._plasticity
    def _set_plasticity(self, bool l):
        proj78._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj78._update_period
    def _set_update_period(self, int l):
        proj78._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj78._update_offset
    def _set_update_offset(self, long l):
        proj78._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj78.get_post_rank()
    def pre_rank_all(self):
        return proj78.get_pre_ranks()
    def pre_rank(self, int n):
        return proj78.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj78.nb_dendrites()
    def nb_synapses(self):
        return proj78.nb_synapses()
    def dendrite_size(self, int n):
        return proj78.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj78.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj78.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj78.size_in_bytes()

    def clear(self):
        return proj78.clear()

# Wrapper for projection 79
@cython.auto_pickle(True)
cdef class proj79_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj79.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj79.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj79.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj79._transmission
    def _set_transmission(self, bool l):
        proj79._transmission = l

    # Update flag
    def _get_update(self):
        return proj79._update
    def _set_update(self, bool l):
        proj79._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj79._plasticity
    def _set_plasticity(self, bool l):
        proj79._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj79._update_period
    def _set_update_period(self, int l):
        proj79._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj79._update_offset
    def _set_update_offset(self, long l):
        proj79._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj79.get_post_rank()
    def pre_rank_all(self):
        return proj79.get_pre_ranks()
    def pre_rank(self, int n):
        return proj79.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj79.nb_dendrites()
    def nb_synapses(self):
        return proj79.nb_synapses()
    def dendrite_size(self, int n):
        return proj79.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj79.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj79.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj79.size_in_bytes()

    def clear(self):
        return proj79.clear()

# Wrapper for projection 80
@cython.auto_pickle(True)
cdef class proj80_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj80.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj80.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj80.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj80._transmission
    def _set_transmission(self, bool l):
        proj80._transmission = l

    # Update flag
    def _get_update(self):
        return proj80._update
    def _set_update(self, bool l):
        proj80._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj80._plasticity
    def _set_plasticity(self, bool l):
        proj80._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj80._update_period
    def _set_update_period(self, int l):
        proj80._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj80._update_offset
    def _set_update_offset(self, long l):
        proj80._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj80.get_post_rank()
    def pre_rank_all(self):
        return proj80.get_pre_ranks()
    def pre_rank(self, int n):
        return proj80.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj80.nb_dendrites()
    def nb_synapses(self):
        return proj80.nb_synapses()
    def dendrite_size(self, int n):
        return proj80.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj80.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj80.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj80.size_in_bytes()

    def clear(self):
        return proj80.clear()

# Wrapper for projection 81
@cython.auto_pickle(True)
cdef class proj81_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj81.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj81.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj81.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj81._transmission
    def _set_transmission(self, bool l):
        proj81._transmission = l

    # Update flag
    def _get_update(self):
        return proj81._update
    def _set_update(self, bool l):
        proj81._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj81._plasticity
    def _set_plasticity(self, bool l):
        proj81._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj81._update_period
    def _set_update_period(self, int l):
        proj81._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj81._update_offset
    def _set_update_offset(self, long l):
        proj81._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj81.get_post_rank()
    def pre_rank_all(self):
        return proj81.get_pre_ranks()
    def pre_rank(self, int n):
        return proj81.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj81.nb_dendrites()
    def nb_synapses(self):
        return proj81.nb_synapses()
    def dendrite_size(self, int n):
        return proj81.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj81.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj81.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj81.size_in_bytes()

    def clear(self):
        return proj81.clear()

# Wrapper for projection 82
@cython.auto_pickle(True)
cdef class proj82_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj82.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj82.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj82.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj82._transmission
    def _set_transmission(self, bool l):
        proj82._transmission = l

    # Update flag
    def _get_update(self):
        return proj82._update
    def _set_update(self, bool l):
        proj82._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj82._plasticity
    def _set_plasticity(self, bool l):
        proj82._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj82._update_period
    def _set_update_period(self, int l):
        proj82._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj82._update_offset
    def _set_update_offset(self, long l):
        proj82._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj82.get_post_rank()
    def pre_rank_all(self):
        return proj82.get_pre_ranks()
    def pre_rank(self, int n):
        return proj82.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj82.nb_dendrites()
    def nb_synapses(self):
        return proj82.nb_synapses()
    def dendrite_size(self, int n):
        return proj82.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj82.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj82.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj82.size_in_bytes()

    def clear(self):
        return proj82.clear()

# Wrapper for projection 83
@cython.auto_pickle(True)
cdef class proj83_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj83.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj83.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj83.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj83._transmission
    def _set_transmission(self, bool l):
        proj83._transmission = l

    # Update flag
    def _get_update(self):
        return proj83._update
    def _set_update(self, bool l):
        proj83._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj83._plasticity
    def _set_plasticity(self, bool l):
        proj83._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj83._update_period
    def _set_update_period(self, int l):
        proj83._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj83._update_offset
    def _set_update_offset(self, long l):
        proj83._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj83.get_post_rank()
    def pre_rank_all(self):
        return proj83.get_pre_ranks()
    def pre_rank(self, int n):
        return proj83.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj83.nb_dendrites()
    def nb_synapses(self):
        return proj83.nb_synapses()
    def dendrite_size(self, int n):
        return proj83.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj83.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj83.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj83.size_in_bytes()

    def clear(self):
        return proj83.clear()

# Wrapper for projection 84
@cython.auto_pickle(True)
cdef class proj84_wrapper :

    def __init__(self, ):
                    pass


    def init_from_lil_connectivity(self, synapses):
        " synapses is an instance of LILConnectivity "
        return proj84.init_from_lil(synapses.post_rank, synapses.pre_rank, synapses.w, synapses.delay)

    def init_from_lil(self, post_rank, pre_rank, w, delay):
        return proj84.init_from_lil(post_rank, pre_rank, w, delay)


    property size:
        def __get__(self):
            return proj84.nb_dendrites()

    # Transmission flag
    def _get_transmission(self):
        return proj84._transmission
    def _set_transmission(self, bool l):
        proj84._transmission = l

    # Update flag
    def _get_update(self):
        return proj84._update
    def _set_update(self, bool l):
        proj84._update = l

    # Plasticity flag
    def _get_plasticity(self):
        return proj84._plasticity
    def _set_plasticity(self, bool l):
        proj84._plasticity = l

    # Update period
    def _get_update_period(self):
        return proj84._update_period
    def _set_update_period(self, int l):
        proj84._update_period = l

    # Update offset
    def _get_update_offset(self):
        return proj84._update_offset
    def _set_update_offset(self, long l):
        proj84._update_offset = l

    # Access connectivity

    def post_rank(self):
        return proj84.get_post_rank()
    def pre_rank_all(self):
        return proj84.get_pre_ranks()
    def pre_rank(self, int n):
        return proj84.get_dendrite_pre_rank(n)
    def nb_dendrites(self):
        return proj84.nb_dendrites()
    def nb_synapses(self):
        return proj84.nb_synapses()
    def dendrite_size(self, int n):
        return proj84.dendrite_size(n)




    # Global Attributes
    def get_global_attribute(self, name, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            return proj84.get_global_attribute_double(cpp_string)


    def set_global_attribute(self, name, value, ctype):
        cpp_string = name.encode('utf-8')

        if ctype == "double":            
            proj84.set_global_attribute_double(cpp_string, value)






        # cuda configuration


    # memory management
    def size_in_bytes(self):
        return proj84.size_in_bytes()

    def clear(self):
        return proj84.clear()


# Monitor wrappers

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder0_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder0.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder0.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder0.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder0.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder0.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder0.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder0.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder0.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder0.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder0.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder0.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder0.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder0.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder0.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder0.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder0.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder0.get_instance(self.id)).r.clear()

    # Targets
# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder1_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder1.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder1.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder1.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder1.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder1.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder1.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder1.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder1.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder1.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder1.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder1.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder1.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder1.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder1.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder1.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder1.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder1.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder1.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder1.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder1.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder1.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder1.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder1.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder1.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder1.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder1.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder1.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder2_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder2.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder2.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder2.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder2.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder2.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder2.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder2.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder2.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder2.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder2.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder2.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder2.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder2.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder2.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder2.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder2.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder2.get_instance(self.id)).r.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder2.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder2.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder2.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder2.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder2.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder2.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder2.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder2.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder2.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder2.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder2.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder2.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder2.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder2.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder2.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder3_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder3.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder3.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder3.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder3.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder3.get_instance(self.id)).r.clear()

    property trace:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).trace
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).record_trace
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (PopRecorder3.get_instance(self.id)).trace.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder3.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder3.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder3.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder3.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder3.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder3.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder3.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder3.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder3.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder3.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder3.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder4_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder4.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder4.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder4.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder4.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder4.get_instance(self.id)).r.clear()

    property trace:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).trace
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).record_trace
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (PopRecorder4.get_instance(self.id)).trace.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder4.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder4.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder4.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder4.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder4.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder4.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder4.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder4.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder4.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder4.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder4.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder5_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder5.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder5.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder5.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder5.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder5.get_instance(self.id)).r.clear()

    property trace:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).trace
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).record_trace
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (PopRecorder5.get_instance(self.id)).trace.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder5.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder5.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder5.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder5.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder5.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder5.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder5.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder5.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder5.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder5.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder5.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder6_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder6.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder6.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder6.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder6.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder6.get_instance(self.id)).r.clear()

    property trace:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).trace
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).record_trace
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (PopRecorder6.get_instance(self.id)).trace.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder6.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder6.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder6.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder6.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder6.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder6.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder6.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder6.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder6.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder6.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder6.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder7_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder7.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder7.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder7.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder7.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder7.get_instance(self.id)).r.clear()

    property trace:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).trace
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).record_trace
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (PopRecorder7.get_instance(self.id)).trace.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder7.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder7.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder7.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder7.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder7.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder7.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder7.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder7.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder7.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder7.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder7.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder8_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder8.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder8.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder8.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).period_offset_ = val

    property ex_in:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).ex_in
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).ex_in = val
    property record_ex_in:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).record_ex_in
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).record_ex_in = val
    def clear_ex_in(self):
        (PopRecorder8.get_instance(self.id)).ex_in.clear()

    property s_inh:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).s_inh
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).s_inh = val
    property record_s_inh:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).record_s_inh
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).record_s_inh = val
    def clear_s_inh(self):
        (PopRecorder8.get_instance(self.id)).s_inh.clear()

    property aux:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).aux
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).aux = val
    property record_aux:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).record_aux
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).record_aux = val
    def clear_aux(self):
        (PopRecorder8.get_instance(self.id)).aux.clear()

    property mp:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder8.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder8.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder8.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder8.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder8.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder8.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder8.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder8.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder8.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder8.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder9_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder9.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder9.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder9.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder9.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder9.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder9.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder9.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder9.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder9.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder9.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder9.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder9.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder9.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder9.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder9.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder9.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder9.get_instance(self.id)).r.clear()

    # Targets
# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder10_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder10.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder10.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder10.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).period_offset_ = val

    property cortex_input:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).cortex_input
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).cortex_input = val
    property record_cortex_input:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).record_cortex_input
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).record_cortex_input = val
    def clear_cortex_input(self):
        (PopRecorder10.get_instance(self.id)).cortex_input.clear()

    property mp:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder10.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder10.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder10.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder10.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder10.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder10.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder10.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder10.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder10.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder10.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder11_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder11.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder11.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder11.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder11.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder11.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder11.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder11.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder11.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder11.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder11.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder11.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder11.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder11.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder11.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder11.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder11.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder11.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder11.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder11.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder11.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder11.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder11.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder11.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder11.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder11.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder11.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder11.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder12_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder12.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder12.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder12.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder12.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder12.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder12.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder12.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder12.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder12.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder12.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder12.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder12.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder12.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder12.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder12.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder12.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder12.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder12.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder12.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder12.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder12.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder12.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder12.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder12.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder12.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder12.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder12.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder13_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder13.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder13.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder13.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder13.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder13.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder13.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder13.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder13.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder13.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder13.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder13.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder13.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder13.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder13.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder13.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder13.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder13.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder13.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder13.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder13.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder13.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder13.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder13.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder13.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder13.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder13.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder13.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder14_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder14.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder14.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder14.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder14.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder14.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder14.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder14.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder14.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder14.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder14.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder14.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder14.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder14.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder14.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder14.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder14.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder14.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder14.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder14.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder14.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder14.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder14.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder14.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder14.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder14.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder14.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder14.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder15_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder15.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder15.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder15.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder15.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder15.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder15.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder15.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder15.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder15.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder15.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder15.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder15.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder15.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder15.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder15.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder15.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder15.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder15.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder15.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder15.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder15.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder15.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder15.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder15.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder15.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder15.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder15.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder16_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder16.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder16.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder16.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder16.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder16.get_instance(self.id)).r.clear()

    property trace:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).trace
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).record_trace
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (PopRecorder16.get_instance(self.id)).trace.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder16.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder16.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder16.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder16.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder16.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder16.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder16.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder16.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder16.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder16.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder16.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder17_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder17.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder17.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder17.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder17.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder17.get_instance(self.id)).r.clear()

    property trace:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).trace
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).record_trace
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (PopRecorder17.get_instance(self.id)).trace.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder17.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder17.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder17.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder17.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder17.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder17.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder17.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder17.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder17.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder17.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder17.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder18_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder18.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder18.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder18.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder18.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder18.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder18.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder18.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder18.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder18.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder18.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder18.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder18.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder18.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder18.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder18.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder18.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder18.get_instance(self.id)).r.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder18.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder18.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder18.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder18.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder18.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder18.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder18.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder18.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder18.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder18.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder18.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder18.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder18.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder18.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder18.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder19_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder19.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder19.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder19.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder19.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder19.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder19.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder19.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder19.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder19.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder19.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder19.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder19.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder19.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder19.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder19.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder19.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder19.get_instance(self.id)).r.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder19.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder19.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder19.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder19.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder19.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder19.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder19.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder19.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder19.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder19.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder19.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder19.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder19.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder19.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder19.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder20_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder20.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder20.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder20.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder20.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder20.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder20.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder20.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder20.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder20.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder20.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder20.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder20.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder20.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder20.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder20.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder20.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder20.get_instance(self.id)).r.clear()

    # Targets
    property _sum_dopa:
        def __get__(self): return (PopRecorder20.get_instance(self.id))._sum_dopa
        def __set__(self, val): (PopRecorder20.get_instance(self.id))._sum_dopa = val
    property record__sum_dopa:
        def __get__(self): return (PopRecorder20.get_instance(self.id)).record__sum_dopa
        def __set__(self, val): (PopRecorder20.get_instance(self.id)).record__sum_dopa = val
    def clear__sum_dopa(self):
        (PopRecorder20.get_instance(self.id))._sum_dopa.clear()

    property _sum_exc:
        def __get__(self): return (PopRecorder20.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder20.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder20.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder20.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder20.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder20.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder20.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder20.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder20.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder20.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder21_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder21.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder21.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder21.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).period_offset_ = val

    property ex_in:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).ex_in
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).ex_in = val
    property record_ex_in:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).record_ex_in
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).record_ex_in = val
    def clear_ex_in(self):
        (PopRecorder21.get_instance(self.id)).ex_in.clear()

    property s_inh:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).s_inh
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).s_inh = val
    property record_s_inh:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).record_s_inh
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).record_s_inh = val
    def clear_s_inh(self):
        (PopRecorder21.get_instance(self.id)).s_inh.clear()

    property aux:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).aux
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).aux = val
    property record_aux:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).record_aux
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).record_aux = val
    def clear_aux(self):
        (PopRecorder21.get_instance(self.id)).aux.clear()

    property mp:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder21.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder21.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder21.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder21.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder21.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder21.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder21.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder21.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder21.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder21.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder22_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder22.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder22.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder22.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder22.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder22.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder22.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder22.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder22.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder22.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder22.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder22.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder22.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder22.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder22.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder22.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder22.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder22.get_instance(self.id)).r.clear()

    # Targets
# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder23_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder23.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder23.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder23.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).period_offset_ = val

    property cortex_input:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).cortex_input
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).cortex_input = val
    property record_cortex_input:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).record_cortex_input
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).record_cortex_input = val
    def clear_cortex_input(self):
        (PopRecorder23.get_instance(self.id)).cortex_input.clear()

    property mp:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder23.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder23.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder23.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder23.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder23.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder23.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder23.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder23.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder23.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder23.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder24_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder24.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder24.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder24.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder24.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder24.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder24.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder24.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder24.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder24.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder24.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder24.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder24.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder24.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder24.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder24.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder24.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder24.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder24.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder24.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder24.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder24.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder24.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder24.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder24.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder24.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder24.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder24.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder25_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder25.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder25.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder25.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder25.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder25.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder25.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder25.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder25.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder25.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder25.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder25.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder25.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder25.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder25.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder25.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder25.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder25.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder25.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder25.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder25.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder25.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder25.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder25.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder25.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder25.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder25.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder25.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder26_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder26.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder26.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder26.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder26.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder26.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder26.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder26.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder26.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder26.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder26.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder26.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder26.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder26.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder26.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder26.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder26.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder26.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder26.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder26.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder26.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder26.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder26.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder26.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder26.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder26.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder26.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder26.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder27_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder27.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder27.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder27.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder27.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder27.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder27.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder27.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder27.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder27.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder27.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder27.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder27.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder27.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder27.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder27.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder27.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder27.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder27.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder27.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder27.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder27.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder27.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder27.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder27.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder27.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder27.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder27.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder28_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder28.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder28.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder28.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder28.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder28.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder28.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder28.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder28.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder28.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder28.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder28.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder28.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder28.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder28.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder28.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder28.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder28.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder28.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder28.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder28.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder28.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder28.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder28.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder28.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder28.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder28.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder28.get_instance(self.id))._sum_inh.clear()

# Population Monitor wrapper
@cython.auto_pickle(True)
cdef class PopRecorder29_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, period_offset, long offset):
        self.id = PopRecorder29.create_instance(ranks, period, period_offset, offset)

    def size_in_bytes(self):
        return (PopRecorder29.get_instance(self.id)).size_in_bytes()

    def clear(self):
        return (PopRecorder29.get_instance(self.id)).clear()

    property period:
        def __get__(self): return (PopRecorder29.get_instance(self.id)).period_
        def __set__(self, val): (PopRecorder29.get_instance(self.id)).period_ = val

    property period_offset:
        def __get__(self): return (PopRecorder29.get_instance(self.id)).period_offset_
        def __set__(self, val): (PopRecorder29.get_instance(self.id)).period_offset_ = val

    property mp:
        def __get__(self): return (PopRecorder29.get_instance(self.id)).mp
        def __set__(self, val): (PopRecorder29.get_instance(self.id)).mp = val
    property record_mp:
        def __get__(self): return (PopRecorder29.get_instance(self.id)).record_mp
        def __set__(self, val): (PopRecorder29.get_instance(self.id)).record_mp = val
    def clear_mp(self):
        (PopRecorder29.get_instance(self.id)).mp.clear()

    property r:
        def __get__(self): return (PopRecorder29.get_instance(self.id)).r
        def __set__(self, val): (PopRecorder29.get_instance(self.id)).r = val
    property record_r:
        def __get__(self): return (PopRecorder29.get_instance(self.id)).record_r
        def __set__(self, val): (PopRecorder29.get_instance(self.id)).record_r = val
    def clear_r(self):
        (PopRecorder29.get_instance(self.id)).r.clear()

    # Targets
    property _sum_exc:
        def __get__(self): return (PopRecorder29.get_instance(self.id))._sum_exc
        def __set__(self, val): (PopRecorder29.get_instance(self.id))._sum_exc = val
    property record__sum_exc:
        def __get__(self): return (PopRecorder29.get_instance(self.id)).record__sum_exc
        def __set__(self, val): (PopRecorder29.get_instance(self.id)).record__sum_exc = val
    def clear__sum_exc(self):
        (PopRecorder29.get_instance(self.id))._sum_exc.clear()

    property _sum_inh:
        def __get__(self): return (PopRecorder29.get_instance(self.id))._sum_inh
        def __set__(self, val): (PopRecorder29.get_instance(self.id))._sum_inh = val
    property record__sum_inh:
        def __get__(self): return (PopRecorder29.get_instance(self.id)).record__sum_inh
        def __set__(self, val): (PopRecorder29.get_instance(self.id)).record__sum_inh = val
    def clear__sum_inh(self):
        (PopRecorder29.get_instance(self.id))._sum_inh.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder0_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder0.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder0.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder0.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder0.get_instance(self.id)).trace.clear()

    property condition_0:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).condition_0
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).condition_0 = val
    property record_condition_0:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).record_condition_0
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).record_condition_0 = val
    def clear_condition_0(self):
        (ProjRecorder0.get_instance(self.id)).condition_0.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder0.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder0.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder0.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder0.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder0.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder1_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder1.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder2_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder2.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder2.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder2.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder2.get_instance(self.id)).trace.clear()

    property aux:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).aux
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).aux = val
    property record_aux:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).record_aux
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).record_aux = val
    def clear_aux(self):
        (ProjRecorder2.get_instance(self.id)).aux.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder2.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder2.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder2.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder2.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder2.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder3_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder3.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder4_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder4.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder5_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder5.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder6_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder6.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder7_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder7.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder8_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder8.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder9_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder9.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder9.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder9.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder9.get_instance(self.id)).trace.clear()

    property condition_0:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).condition_0
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).condition_0 = val
    property record_condition_0:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).record_condition_0
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).record_condition_0 = val
    def clear_condition_0(self):
        (ProjRecorder9.get_instance(self.id)).condition_0.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder9.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder9.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder9.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder9.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder9.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder10_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder10.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder11_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder11.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder11.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder11.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder11.get_instance(self.id)).trace.clear()

    property aux:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).aux
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).aux = val
    property record_aux:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).record_aux
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).record_aux = val
    def clear_aux(self):
        (ProjRecorder11.get_instance(self.id)).aux.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder11.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder11.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder11.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder11.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder11.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder12_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder12.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder13_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder13.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder13.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder13.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder13.get_instance(self.id)).trace.clear()

    property condition_0:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).condition_0
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).condition_0 = val
    property record_condition_0:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).record_condition_0
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).record_condition_0 = val
    def clear_condition_0(self):
        (ProjRecorder13.get_instance(self.id)).condition_0.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder13.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder13.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder13.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder13.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder13.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder14_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder14.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder15_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder15.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder15.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder15.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder15.get_instance(self.id)).trace.clear()

    property aux:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).aux
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).aux = val
    property record_aux:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).record_aux
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).record_aux = val
    def clear_aux(self):
        (ProjRecorder15.get_instance(self.id)).aux.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder15.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder15.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder15.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder15.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder15.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder16_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder16.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder17_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder17.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder17.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder17.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder17.get_instance(self.id)).trace.clear()

    property condition_0:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).condition_0
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).condition_0 = val
    property record_condition_0:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).record_condition_0
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).record_condition_0 = val
    def clear_condition_0(self):
        (ProjRecorder17.get_instance(self.id)).condition_0.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder17.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder17.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder17.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder17.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder17.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder18_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder18.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder19_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder19.create_instance(ranks, period, period_offset, offset)

    property aux:
        def __get__(self): return (ProjRecorder19.get_instance(self.id)).aux
        def __set__(self, val): (ProjRecorder19.get_instance(self.id)).aux = val
    property record_aux:
        def __get__(self): return (ProjRecorder19.get_instance(self.id)).record_aux
        def __set__(self, val): (ProjRecorder19.get_instance(self.id)).record_aux = val
    def clear_aux(self):
        (ProjRecorder19.get_instance(self.id)).aux.clear()

    property delta:
        def __get__(self): return (ProjRecorder19.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder19.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder19.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder19.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder19.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder19.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder19.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder19.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder19.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder19.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder20_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder20.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder21_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder21.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder22_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder22.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder23_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder23.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder24_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder24.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder25_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder25.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder26_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder26.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder27_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder27.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder28_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder28.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder29_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder29.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder30_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder30.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder31_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder31.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder32_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder32.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder33_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder33.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder34_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder34.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder35_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder35.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder36_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder36.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder36.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder36.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder36.get_instance(self.id)).trace.clear()

    property condition_0:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).condition_0
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).condition_0 = val
    property record_condition_0:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).record_condition_0
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).record_condition_0 = val
    def clear_condition_0(self):
        (ProjRecorder36.get_instance(self.id)).condition_0.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder36.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder36.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder36.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder36.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder36.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder37_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder37.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder38_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder38.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder38.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder38.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder38.get_instance(self.id)).trace.clear()

    property aux:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).aux
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).aux = val
    property record_aux:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).record_aux
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).record_aux = val
    def clear_aux(self):
        (ProjRecorder38.get_instance(self.id)).aux.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder38.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder38.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder38.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder38.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder38.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder39_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder39.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder40_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder40.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder41_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder41.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder42_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder42.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder43_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder43.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder44_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder44.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder45_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder45.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder46_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder46.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder47_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder47.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder48_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder48.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder49_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder49.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder49.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder49.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder49.get_instance(self.id)).trace.clear()

    property condition_0:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).condition_0
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).condition_0 = val
    property record_condition_0:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).record_condition_0
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).record_condition_0 = val
    def clear_condition_0(self):
        (ProjRecorder49.get_instance(self.id)).condition_0.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder49.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder49.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder49.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder49.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder49.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder50_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder50.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder51_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder51.create_instance(ranks, period, period_offset, offset)

    property alpha:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).alpha
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).alpha = val
    property record_alpha:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).record_alpha
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).record_alpha = val
    def clear_alpha(self):
        (ProjRecorder51.get_instance(self.id)).alpha.clear()

    property dopa_sum:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).dopa_sum
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).dopa_sum = val
    property record_dopa_sum:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).record_dopa_sum
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).record_dopa_sum = val
    def clear_dopa_sum(self):
        (ProjRecorder51.get_instance(self.id)).dopa_sum.clear()

    property trace:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).trace
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).trace = val
    property record_trace:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).record_trace
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).record_trace = val
    def clear_trace(self):
        (ProjRecorder51.get_instance(self.id)).trace.clear()

    property aux:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).aux
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).aux = val
    property record_aux:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).record_aux
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).record_aux = val
    def clear_aux(self):
        (ProjRecorder51.get_instance(self.id)).aux.clear()

    property dopa_mod:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).dopa_mod
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).dopa_mod = val
    property record_dopa_mod:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).record_dopa_mod
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).record_dopa_mod = val
    def clear_dopa_mod(self):
        (ProjRecorder51.get_instance(self.id)).dopa_mod.clear()

    property delta:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder51.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder51.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder51.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder51.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder52_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder52.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder53_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder53.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder54_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder54.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder55_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder55.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder56_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder56.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder57_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder57.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder58_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder58.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder59_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder59.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder60_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder60.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder61_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder61.create_instance(ranks, period, period_offset, offset)

    property aux:
        def __get__(self): return (ProjRecorder61.get_instance(self.id)).aux
        def __set__(self, val): (ProjRecorder61.get_instance(self.id)).aux = val
    property record_aux:
        def __get__(self): return (ProjRecorder61.get_instance(self.id)).record_aux
        def __set__(self, val): (ProjRecorder61.get_instance(self.id)).record_aux = val
    def clear_aux(self):
        (ProjRecorder61.get_instance(self.id)).aux.clear()

    property delta:
        def __get__(self): return (ProjRecorder61.get_instance(self.id)).delta
        def __set__(self, val): (ProjRecorder61.get_instance(self.id)).delta = val
    property record_delta:
        def __get__(self): return (ProjRecorder61.get_instance(self.id)).record_delta
        def __set__(self, val): (ProjRecorder61.get_instance(self.id)).record_delta = val
    def clear_delta(self):
        (ProjRecorder61.get_instance(self.id)).delta.clear()

    property w:
        def __get__(self): return (ProjRecorder61.get_instance(self.id)).w
        def __set__(self, val): (ProjRecorder61.get_instance(self.id)).w = val
    property record_w:
        def __get__(self): return (ProjRecorder61.get_instance(self.id)).record_w
        def __set__(self, val): (ProjRecorder61.get_instance(self.id)).record_w = val
    def clear_w(self):
        (ProjRecorder61.get_instance(self.id)).w.clear()

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder62_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder62.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder63_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder63.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder64_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder64.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder65_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder65.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder66_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder66.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder67_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder67.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder68_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder68.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder69_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder69.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder70_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder70.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder71_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder71.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder72_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder72.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder73_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder73.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder74_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder74.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder75_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder75.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder76_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder76.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder77_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder77.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder78_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder78.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder79_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder79.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder80_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder80.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder81_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder81.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder82_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder82.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder83_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder83.create_instance(ranks, period, period_offset, offset)

# Projection Monitor wrapper
@cython.auto_pickle(True)
cdef class ProjRecorder84_wrapper:
    cdef int id
    def __init__(self, list ranks, int period, int period_offset, long offset):
        self.id = ProjRecorder84.create_instance(ranks, period, period_offset, offset)


# User-defined functions


# User-defined constants


# Initialize the network
def pyx_create(double dt):
    initialize(dt)

# Simple progressbar on the command line
def progress(count, total, status=''):
    """
    Prints a progress bar on the command line.

    adapted from: https://gist.github.com/vladignatyev/06860ec2040cb497f0f3

    Modification: The original code set the '\r' at the end, so the bar disappears when finished.
    I moved it to the front, so the last status remains.
    """
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    sys.stdout.write('\r[%s] %s%s ...%s' % (bar, percents, '%', status))
    sys.stdout.flush()

# Simulation for the given number of steps
def pyx_run(int nb_steps, progress_bar):
    cdef int nb, rest
    cdef int batch = 1000
    if nb_steps < batch:
        with nogil:
            run(nb_steps)
    else:
        nb = int(nb_steps/batch)
        rest = nb_steps % batch
        for i in range(nb):
            with nogil:
                run(batch)
            PyErr_CheckSignals()
            if nb > 1 and progress_bar:
                progress(i+1, nb, 'simulate()')
        if rest > 0:
            run(rest)

        if (progress_bar):
            print('\n')

# Simulation for the given number of steps except if a criterion is reached
def pyx_run_until(int nb_steps, list populations, bool mode):
    cdef int nb
    nb = run_until(nb_steps, populations, mode)
    return nb

# Simulate for one step
def pyx_step():
    step()

# Access time
def set_time(t):
    setTime(t)
def get_time():
    return getTime()

# Access dt
def set_dt(double dt):
    setDt(dt)
def get_dt():
    return getDt()


# Set number of threads
def set_number_threads(int n, core_list):
    setNumberThreads(n, core_list)


# Set seed
def set_seed(long seed, int num_sources, use_seed_seq):
    setSeed(seed, num_sources, use_seed_seq)
