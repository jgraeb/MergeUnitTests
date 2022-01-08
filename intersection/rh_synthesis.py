## Apurva Badithela and Josefine Graebener
# Receding horizon winning set synthesis for intersection

from graph_construction import *
import numpy as np
from specifications import *
from tools import WinningSet
 #check_all_states_in_winset_rh, check_state_in_fp
from graph_construction import flip_state_dictionaries, set_up_partial_order_for_rh
import pdb

PRINT_STATES_IN_COMPUTATION = True
FILTER_FIXPOINT = False


# Function to get specifications for receeding horizon synthesis:
# Base specification
def rh_base_spec():
    intersectionfile = 'intersectionfile.txt'
    state_dict, crosswalk = create_intersection_from_file(intersectionfile)

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
def construct_spec_set_membership(Vj, sys_st2ver_dict, test_st2ver_dict):
    sys_ver2st_dict, test_ver2st_dict = flip_state_dictionaries(sys_st2ver_dict, test_st2ver_dict)
    spec = "("
    for vj in Vj:
        if vj in sys_ver2st_dict:
            state_tup = sys_ver2st_dict[vj] ### Modify this line. Depending on vj, need to decide between sys_st2ver_dict or test_st2ver_dict
        elif vj in test_ver2st_dict:
            state_tup = test_ver2st_dict[vj]
        state_dict = make_dict_from_tuple(state_tup)
        if spec == "(":
            spec += get_str_spec(state_dict)
        else:
            spec += " || " + get_str_spec(state_dict)
    spec += ")"
    return spec

def make_dict_from_tuple(tuple):
    out_dict = dict()
    out_dict = {'y': tuple[0][0], 'z': tuple[0][1], 'y1': tuple[1][0], 'z1': tuple[1][1], 'p': tuple[-1]}
    return out_dict

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

# Function to generate winning sets with receding horizon approach
def rh_winsets(Vij, G_aux, sys_st2ver_dict, test_st2ver_dict):
    jmax = len(Vij) - 1
    Wij = dict()
    for j in np.linspace(jmax, 0, jmax+1):
        if j%2 == 0:
            test_rh_spec, ego_rh_spec, goal_states = rh_spec_add_progress(Vij, j, sys_st2ver_dict, test_st2ver_dict)
            W, fixpt, aut = find_winset(test_rh_spec, ego_rh_spec)

            pdb.set_trace()

            states_in_fp, states_out_fp = check_all_states_in_fp(W, fixpt, aut, sys_st2ver_dict, test_st2ver_dict)
            if PRINT_STATES_IN_COMPUTATION:
                print(" ")
                print("Printing states in winning set: ")

            if FILTER_FIXPOINT:
                start_set = Vij[j]
                states_in_winset, states_out_winset = check_all_states_in_winset_rh(W, fixpt, aut, goal_states, G_aux, sys_st2ver_dict, test_st2ver_dict, start_set)
                Wij.update({j: states_in_winset})
            else:
                Wij.update({j: states_in_winset})
    return Wij

# Function to get the winning sets for all states
def get_states_in_rh_winsets(Vij, G_aux, sys_st2ver_dict, test_st2ver_dict):
    """
    Find all tester states in the winning sets for each goal
    W - Winning set
    indices:
    i - index of the goal
    j - distance to corresponding goal in steps
    """

    Wij = dict()
    for key in Vij.keys():
        Wj = rh_winsets(Vij[key], G_aux, sys_st2ver_dict, test_st2ver_dict)
        Wij.update({key: Wj})
    return Wij


if __name__ == '__main__':
    Vij, G_aux, sys_st2ver_dict, test_st2ver_dict = set_up_partial_order_for_rh()
    Wij = get_states_in_rh_winsets(Vij, G_aux, sys_st2ver_dict, test_st2ver_dict)
