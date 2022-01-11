## Apurva Badithela and Josefine Graebener
# Receding horizon winning set synthesis for intersection
import numpy as np
import pdb
# from graph_construction import flip_state_dictionaries, set_up_partial_order_for_rh
from intersection.graph_construction import *
from intersection.specifications import *
from intersection.tools import WinningSet, check_all_states_in_winset, check_all_states_in_fp, convert_tuple2dict, synthesize_some_controller, check_assumptions


PRINT_STATES_IN_COMPUTATION = False
FILTER_FIXPOINT = True
VERIFY_W = False

# Function to get specifications for receeding horizon synthesis:
# Base specification
def rh_base_spec():
    intersectionfile = 'intersection/intersectionfile.txt'
    state_dict, crosswalk = create_intersection_from_file(intersectionfile)

    sys_vars, y_min_grid, y_max_grid, z_min_grid, z_max_grid = sys_variables()
    sys_init = initial_sys_vars()
    sys_prog = progress_sys_vars()
    sys_safe = set()

    # add the dynamics for the system
    sys_safe |= dynamics_system(state_dict,[('y','z')], y_min_grid,y_max_grid, z_min_grid,z_max_grid)
    # st()
    sys_safe |= intersection_clear_eventually_system_drives(state_dict, y_min_grid, y_max_grid, z_min_grid, z_max_grid)
    y_val = 3
    z_min = 0
    z_max = 7
    agent_var = ['y','z']
    sys_safe |= once_system_entered_intersection_keep_driving(state_dict, agent_var, y_val, z_min, z_max)

    # tester car + pedestrian
    # initial positions
    y_min_tester = 0
    y_max_tester = 7
    z_min_tester = 3
    z_max_tester = 3
    tester_vars, min_cw, max_cw = tester_variables(y_min_tester, y_max_tester, z_min_tester, z_max_tester)
    tester_init = init_tester_vars()

    tester_prog = set()
    tester_safe = set()

    # Add the dynamics
    tester_safe |= dynamics_tester_car(state_dict, ('y1','z1'), y_min_tester, y_max_tester, z_min_tester, z_max_tester)
    # st()
    tester_safe |= dynamics_ped('p', min_cw, max_cw)

    # Add no collissions between any agents
    no_collisions_tester, no_collisions_sys = collision_safety(y_min_tester, y_max_tester, z_min_tester, z_max_tester, crosswalk)
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
        vj = check_vj_string(vj) # Converts string to numeric
        if vj in sys_ver2st_dict.keys():
            state_tup = sys_ver2st_dict[vj] ### Modify this line. Depending on vj, need to decide between sys_st2ver_dict or test_st2ver_dict
        elif vj in test_ver2st_dict.keys():
            state_tup = test_ver2st_dict[vj]
        else:
            raise Exception("state_tup not set!")

        state_dict = make_dict_from_tuple(state_tup)
        if spec == "(":
            spec += get_str_spec(state_dict)
        else:
            spec += " || " + get_str_spec(state_dict)
    spec += ")"
    return spec

# If vj is a node in the auxiliary graph, this function converts it back:
def check_vj_string(vj):
    vj_str = str(vj) # Converts to string
    vj_split = vj_str.split('_') # split at _, if any
    vj_new = int(vj_split[0])
    return vj_new

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
    #st()
    test_spec.prog |= prog_guarantee
    ego_spec.safety |= assumption
    return test_spec, ego_spec, goal_states

# Construct receding horizon winning set specifications:
def find_winset(test_spec, ego_spec):
    gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
    w_set = WinningSet()
    w_set.set_spec(gr_spec)
    aut = w_set.make_compatible_automaton(gr_spec)
    g = synthesize_some_controller(aut)
    fp = w_set.find_winning_set(aut)
    return w_set, fp, aut

# Test specification:
def test_intersection_spec(G_aux, sys_st2ver_dict, test_st2ver_dict):
    intersectionfile = 'intersection/intersectionfile.txt'
    state_dict, crosswalk = create_intersection_from_file(intersectionfile)
    ego_spec, test_spec = intersection_specs(state_dict, crosswalk)
    # st()
    W, fixpt, aut = find_winset(test_spec, ego_spec)
    states_in_fp, states_out_fp = check_all_states_in_fp(W, fixpt, aut, sys_st2ver_dict, test_st2ver_dict)
    states_in_W, states_out_W = check_all_states_in_winset(states_in_fp)
    print("States in original fixpoint: ")
    print(len(states_in_fp))
    print("States in winning set: ")
    print(len(states_in_W))

    # Verify winning set:
    verify_winset(states_in_W, test_spec, ego_spec, type="in_W")
    verify_winset(states_out_fp, test_spec, ego_spec, type="out_W")

    #st()

# Function to verify winset:
def verify_winset(states, test_spec, ego_spec, type="in_W"):
    if type=="in_W":
        counterexamples_in = verify_W(states, test_spec, ego_spec, type="in_W")
        print("No. of ounterexamples in W: ")
        print(len(counterexamples_in))
    elif type=="out_W":
        counterexamples_out = verify_W(states, test_spec, ego_spec, type="out_W")
        print("No. of ounterexamples out W: ")
        print(len(counterexamples_out))
    else:
        print("Type should be in_W or out_W")

# Function to generate winning sets with receding horizon approach
def rh_winsets(Vij, G_aux, sys_st2ver_dict, test_st2ver_dict):
    jmax = len(Vij) - 1
    Wij = dict()
    for j in np.linspace(jmax, 0, jmax+1):
        if j%2 == 0:
            test_rh_spec, ego_rh_spec, goal_states = rh_spec_add_progress(Vij, j, sys_st2ver_dict, test_st2ver_dict)
            W, fixpt, aut = find_winset(test_rh_spec, ego_rh_spec)
            states_in_fp, states_out_fp = check_all_states_in_fp(W, fixpt, aut, sys_st2ver_dict, test_st2ver_dict)
            # st()
            if PRINT_STATES_IN_COMPUTATION:
                print(" ")
                print("Printing states in winning set: ")

            if FILTER_FIXPOINT:
                start_set_Gaux = Vij[j] # String form with Gaux vertices
                start_set = [check_vj_string(vj) for vj in start_set_Gaux]
                # st()
                sys_ver2st_dict, test_ver2st_dict = flip_state_dictionaries(sys_st2ver_dict, test_st2ver_dict)
                states_in_winset, states_out_winset = check_all_states_in_winset(states_in_fp, sys_ver2st_dict, test_ver2st_dict, start_set)
                Wij.update({j: states_in_winset})
                # st()
                # Verifying the winset
                if VERIFY_W:
                    ego_spec, test_spec = rh_base_spec()
                    ego_spec.init = set()
                    test_spec.init = set()
                    verify_winset(states_in_winset, test_spec, ego_spec, type="in_W")
                    verify_winset(states_out_fp, test_spec, ego_spec, type="out_W")
            else:
                Wij.update({j: states_in_fp})


    return Wij

# Verify winset:
# Function to verify that the winning set is correct:
def verify_W(list_states, test_spec, ego_spec, type="in_W"):
    counterexamples = []
    for state in list_states:
        state_dict = convert_tuple2dict(state)
        # Set initial conditions to present state and seeing if a controller can be synthesized:
        test_spec.init = {'y1 = ' + str(state_dict['y1']), 'z1 = ' + str(state_dict['z1']), 'p=' + str(state_dict['p'])}
        ego_spec.init = {'y = '+ str(state_dict['y']), 'z = ' + str(state_dict['z'])}
        gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
        w_set = WinningSet()
        w_set.set_spec(gr_spec)
        aut = w_set.make_compatible_automaton(gr_spec)
        if type == "in_W":
            try:
                g = synthesize_some_controller(aut)
            except:
                print('Found counterexample in W')
                counterexamples.append(state)
        elif type == "out_W":
            try:
                g = synthesize_some_controller(aut)
                counterexamples.append(state)
                print('Found counterexample out W')
            except:
                pass
        else:
            pass
    return counterexamples

# Not all goals are valid according to specs. Function to sieve out bad goals:
def check_Vij_goals(init_goals, sys_st2ver_dict, test_st2ver_dict):
    proper_goals = []
    sys_ver2st_dict, test_ver2st_dict = flip_state_dictionaries(sys_st2ver_dict, test_st2ver_dict)
    for g in init_goals:
        state_st = check_vj_string(g)
        state_tup = test_ver2st_dict[state_st]
        good_goal = check_assumptions(make_dict_from_tuple(state_tup))
        if good_goal:
            proper_goals.append(g)
        else:
            pass
            # print("Bad goal: ")
            # print(state_tup)
    return proper_goals

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
    proper_goals = check_Vij_goals(list(Vij.keys()), sys_st2ver_dict, test_st2ver_dict) # Not all goals are valid according to specs
    for key in Vij.keys():
        if key in proper_goals:
            Wj = rh_winsets(Vij[key], G_aux, sys_st2ver_dict, test_st2ver_dict)
            Wij.update({key: Wj})
            # st()

            # for k in Wj.keys():
            #     print(k)
            #     print(Wj[k])

    return Wij

def synthesize_intersection_filter():
    Vij, G_aux, sys_st2ver_dict, test_st2ver_dict = set_up_partial_order_for_rh()
    Wij = get_states_in_rh_winsets(Vij, G_aux, sys_st2ver_dict, test_st2ver_dict)
    # st()
    return Wij, Vij, G_aux, sys_st2ver_dict, test_st2ver_dict

if __name__ == '__main__':
    Vij, G_aux, sys_st2ver_dict, test_st2ver_dict = set_up_partial_order_for_rh()
    # st()
    test_intersection_spec(G_aux, sys_st2ver_dict, test_st2ver_dict)
    Wij = get_states_in_rh_winsets(Vij, G_aux, sys_st2ver_dict, test_st2ver_dict)
