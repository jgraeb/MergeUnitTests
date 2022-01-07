## Apurva Badithela and Josefine Graebener
# Receding horizon winning set synthesis for intersection

from graph_construction import *
import numpy as np
from tools import WinningSet, check_all_states_in_winset_rh, check_state_in_fp

PRINT_STATES_IN_COMPUTATION = True
FILTER_FIXPOINT = False
# Function to get specifications for receeding horizon synthesis:
# Base specification
def rh_base_spec():
    sys_vars, y_min_grid, y_max_grid, z_min_grid, z_max_grid = sys_variables()
    sys_init = initial_sys_vars()
    sys_prog = progress_sys_vars()
    sys_safe = set()

    # add the dynamics for the system
    sys_safe |= dynamics_car(state_dict,[('y','z')], y_min_grid,y_max_grid, z_min_grid,z_max_grid)

    # tester car + pedestrian
    # initial positions
    tester_vars, min_cw, max_cw = tester_variables(y_min_grid, y_max_grid, z_min_grid, z_max_grid)
    tester_init = init_tester_vars()

    tester_prog = set()
    tester_safe = set()

    # Add the dynamics
    tester_safe |= dynamics_car(state_dict, [('y1','z1')], y_min_grid, y_max_grid, z_min_grid, z_max_grid)
    tester_safe |= dynamics_ped('p', min_cw, max_cw)

    # Add no collissions between any agents
    no_collisions_tester, no_collisions_sys = collision_safety(y_min_grid, y_max_grid, z_min_grid, z_max_grid, crosswalk)
    tester_safe |= no_collisions_tester
    sys_safe |= no_collisions_sys

    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    return ego_spec, test_spec

# Define string for state specification:
# Find next state
def get_str_spec(state_dict):
    spec_state = ""
    for k, v in state_dict.items(): # Variables describing states of the system and environment
        if spec_state == "":
            spec_state += "(" + k + " = " + str(v)
        else:
            spec_state += " && " + k + " = " + str(v)
    spec_state += ")"
    return spec_state

# Function to construct set membership:
#
def construct_spec_set_membership(Vj, sys_st2ver_dict, test_st2ver_dict):
    spec = "("
    for vj in Vj:
        state_dict = ver2st_dict[vj] ### Modify this line. Depending on vj, need to decide between sys_st2ver_dict or test_st2ver_dict
        if spec == "(":
            spec += get_str_spec(state_dict)
        else:
            spec += " || " + get_str_spec(state_dict)
    spec += ")"
    return spec

# Function to add progress properties of the jth specification for the ith goal:
# Keep separate goals for each winning set
# i: goal
# Vij_dict: Dictionary
# j: Distance from the goal
def add_psi_i_j_progress(Vij_dict, j, sys_st2ver_dict, test_st2ver_dict):
    assumption = set()
    prog_guarantee = set()
    Vj = Vij_dict[j]
    assert j%2 == 0 # Make sure j is odd
    if j >= 2:
        FVj = Vij_dict[j-2]
    else:
        FVj = Vij_dict[0]

    assumption_spec = construct_spec_set_membership(Vj, sys_st2ver_dict, test_st2ver_dict)
    assumption |= {assumption_spec}

    prog_spec = construct_spec_set_membership(FVj, sys_st2ver_dict, test_st2ver_dict)
    prog_guarantee |= {prog_spec}

    return assumption, prog_guarantee, FVj

# Add progress specifications to base spec:
# Make sure inital conditions are empty
def rh_spec_add_progress(Vij, j, sys_st2ver_dict, test_st2ver_dict):
    ego_spec, test_spec = rh_base_spec()
    ego_spec.init = set()
    test_spec.init = set()
    assumption, prog_guarantee, goal_states = add_psi_i_j_progress(Vij, j, sys_st2ver_dict, test_st2ver_dict)
    test_spec.prog |= prog_guarantee
    ego_spec.safety |= assumption
    return test_spec, ego_spec, goal_states

# Construct receding horizon winning set specifications:
def find_winset(test_spec, ego_spec):
    gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
    w_set = WinningSet()
    w_set.set_spec(gr_spec)
    aut = w_set.make_compatible_automaton(gr_spec)
    # g = synthesize_some_controller(aut)
    fp = w_set.find_winning_set(aut)
    return w_set, fp, aut

def get_tester_states_in_winsets(tracklength, merge_setting):
    """
    Find all tester states in the winning sets.
    W - Winning set
    indices:
    i - index of the goal
    j - distance to corresponding goal in steps
    """
    ego_spec, test_spec, Vij_dict, state_tracker, ver2st_dict, G, state_dict_test, state_dict_system = specs_car_rh(tracklength, merge_setting)
    Wij = dict()
    for key in Vij_dict.keys():
        Wj = get_winset_rh(tracklength, merge_setting, Vij_dict[key], state_tracker, ver2st_dict,ego_spec, test_spec, state_dict_test, state_dict_system, G)
        Wij.update({key: Wj})
    return Wij, Vij_dict, state_tracker, ver2st_dict

# Function to generate winning sets with receding horizon approach
def rh_winsets(Vij, sys_st2ver_dict, test_st2ver_dict):
    jmax = len(Vij) - 1
    Wij = dict()
    for j in np.linspace(jmax, 0, jmax+1):
        if j%2 == 0:
            test_rh_spec, ego_rh_spec, goal_states = rh_spec_add_progress(Vij, j, sys_st2ver_dict, test_st2ver_dict)
            W, fixpt, aut = find_winset(test_rh_spec, ego_rh_spec)

            states_in_fp, states_out_fp = check_all_states_in_fp(W, fixpt, aut)
            if PRINT_STATES_IN_COMPUTATION:
                print(" ")
                print("Printing states in winning set: ")

            if FILTER_FIXPOINT:
                start_set = Vij[j]
                states_in_winset, states_out_winset = check_all_states_in_winset_rh(W, fixpt, aut, goal_states, G, sys_st2ver_dict, test_st2ver_dict, start_set)
                Wij.update({j: states_in_winset})
            else:
                Wij.update({j: states_in_winset})
