import sys
sys.path.append('..')
import tulip as tulip
import numpy as np
from tulip.spec import form
import networkx as nx
from omega.symbolic import temporal as trl
import _pickle as pickle
import os
from omega.games import enumeration as enum
from omega.symbolic import enumeration as sym_enum
from copy import deepcopy
from ipdb import set_trace as st
from omega.games import gr1
from omega.logic import syntax as stx
from omega.symbolic import fixpoint as fx
from omega.symbolic import fol as _fol
from omega.symbolic import prime as prm
from tulip.interfaces.omega import _grspec_to_automaton, _strategy_to_state_annotated
from tulip import spec
from tulip import transys
from tulip.synth import sys_to_spec
import logging
from copy import deepcopy
from ipdb import set_trace as st
from tulip import transys, spec, synth
from winning_set.correct_win_set import get_winset, WinningSet, make_grspec, check_all_states_in_fp, check_all_states_in_winset, Spec
import networkx as nx
from intersection import make_state_dictionary_for_specification
from abstraction_intersection import get_graph, create_intersection_from_file

# Function to collect Wj for each goal:
# Depending on the goal, there is a different ordering of Wj's of for each goal i.
# This function returns a dictionary of the set of Wjs for each goal
# Intuition: This is similar to specifying the terminal condition in MPC
# Depending on states that satisfy the goal_lambda condition, those are added to the Wj_dict
# Get Vj for all goals
def get_Vj_for_all_goals(tracklength, G, st2ver_dict, ver2st_dict, state_tracker, goal_lambda):
    """
    Parameters
    ----------
    tracklength : TYPE
        DESCRIPTION.
    G : TYPE
        DESCRIPTION.
    st2ver_dict : TYPE
        DESCRIPTION.
    ver2st_dict : TYPE
        DESCRIPTION.
    goal_lambda : TYPE
        DESCRIPTION.
    Returns
    -------
    Wj_dict : TYPE
        DESCRIPTION.
    """
    Vj_dict = dict()
    goal_nodes = get_goal_states(G, goal_lambda, ver2st_dict, state_tracker)
    for i in goal_nodes:
        Vj_dict[i] = get_Vj(tracklength, G, st2ver_dict, ver2st_dict, i)
    return Vj_dict

# Function to get Vi_j, which is the set of all states that are j*horizon steps
# (1 step = 1 round of play) away from set from progress goal []<>i:
def get_Vj(tracklength, G, st2ver_dict, ver2st_dict, goal):
    """
    Parameters
    ----------
    tracklength : TYPE
        DESCRIPTION.
    G : TYPE
        DESCRIPTION.
    st2ver_dict : TYPE
        DESCRIPTION.
    ver2st_dict : TYPE
        DESCRIPTION.
    goal : TYPE
        DESCRIPTION.
    Returns
    -------
    Vj_set : TYPE
        DESCRIPTION.
    """
    # G_aux, target = add_aux_nodes(G, goal_state)
    node_dist = dict()
    Vj_set = dict()
    for node in list(G.nodes):
        if nx.has_path(G, node, goal):
            pl = shortest_path_length(G, node, goal)
            node_dist[node] = pl
            if pl not in list(Vj_set.keys()):
                Vj_set[pl] = [node]
            else:
                Vj_set[pl].append(node)
    return Vj_set


# Ego safe controller for left turn requirement:
def get_ego_safety():
    ego_safe = set()
    # Fill in ego safety specifications:
    return ego_safe

def add_psi_i_j_progress(Vij_dict, j, ver2st_dict):
    '''
    Function to add progress properties of the jth specification for the ith goal. Keep separate goals for each winning set.
    Vij_dict: Set of all Vjs for goal i
    j: Distance to goal i
    ver2st_dict: Dictionary going from number to state dictionary
    '''
    assumption = set()
    prog_guarantee = set()
    Vj = Vij_dict[j]
    assert j%2 == 0 # Make sure j is odd
    if j >= 2:
        FVj = Vij_dict[j-2]
    else:
        FVj = Vij_dict[0]

    assumption_spec = construct_spec_set_membership(Vj, ver2st_dict)
    assumption |= {assumption_spec}

    prog_spec = construct_spec_set_membership(FVj, ver2st_dict)
    prog_guarantee |= {prog_spec}

    return assumption, prog_guarantee, FVj

def get_str_spec(state_dict):
    '''
    Helper function that define string from state_dict
    '''
    spec_state = ""
    for k, v in state_dict.items():
        if spec_state == "":
            spec_state += "(" + k + " = " + str(v)
        else:
            spec_state += " && " + k + " = " + str(v)
    spec_state += ")"
    return spec_state

def construct_spec_set_membership(Vj, ver2st_dict):
    '''
    Helper function that constructs set membership
    '''
    spec = "("
    for vj in Vj:
        state_dict = ver2st_dict[vj]
        if spec == "(":
            spec += get_str_spec(state_dict)
        else:
            spec += " || " + get_str_spec(state_dict)
    spec += ")"
    return spec

def get_test_safety():
    '''
    Function to specify safety specifications for the test environments
    Parameters
    ----------

    Returns
    -------
    test_safe : set(str)
        A set of safety formulae that the testers need to follow.
        This function returns the dynamics of the test agents
    '''
    test_safe = set()
    return test_safe

# Function to add merge specifications:
def add_goal_specs(test_safe):
    """
    Goal specifications for the intersection example
    Parameters
    ----------
    test_safe : Set of initial safety specifications
        DESCRIPTION.
    Returns
    -------
    test_safe : Set of safety specifications with goal requirements added in
        DESCRIPTION.
    """
    goal_spec = ''
    test_safe |= {goal_spec}
    return test_safe

# Function to get all possible states in the graph of all system and environment transitions:
def specs_car_rh():
    ego_vars = {}
    ego_vars['x'] = (1, tracklength)
    ego_vars['y'] = (1,2)
    ego_init = {'x='+str(1), 'y='+str(1)}
    ego_safe = get_ego_safety(tracklength, merge_setting)
    ego_prog = {'y=2'}

    test_vars = {}
    test_vars['x1'] = (1, tracklength)
    test_vars['x2'] = (1, tracklength)
    test_vars['y1'] = (1,2)
    test_vars['y2'] = (1,2)
    test_init = {'x1='+str(2), 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    test_safe = set()
    test_safe = get_test_safety(tracklength, merge_setting)
    test_safe = add_merge_specs(tracklength, merge_setting, test_safe)
    test_prog = set()
    # test_prog = add_psi_j_progress(tracklength, j, merge_setting)

    ego_n = 2*tracklength
    tester_n = (tracklength-1)**2
    G, st2ver_dict, ver2st_dict, state_tracker, state_dict_test, state_dict_system = get_all_states(tracklength, ego_n, tester_n)
    merge_setting = "between"
    # pdb.set_trace()
    goal_lambda = construct_lambda_function(merge_setting)
    Vij_dict = get_Vj_for_all_goals(tracklength, G, st2ver_dict, ver2st_dict, state_tracker, goal_lambda)
    # pdb.set_trace()

    ego_spec = Spec(ego_vars, ego_init, ego_safe, ego_prog)
    test_spec = Spec(test_vars, test_init, test_safe, test_prog)
    return ego_spec, test_spec, Vij_dict, state_tracker, ver2st_dict, G, state_dict_test, state_dict_system

# Function to get all receding horizon winning sets:
# tracklength: length of the track; merge_setting: between/ in front/ behind
def get_winset_rh(tracklength, merge_setting, Vij, state_tracker, ver2st_dict,ego_spec_orginal, test_spec_orginal, state_test_dict, state_system_dict, G):
    # Get Wij_dict: The list of Vij
    # Existing safety, progress and init properties
    # ego_spec, test_spec, Vij_dict, state_tracker, ver2st_dict = specs_car_rh(tracklength, merge_setting)

    # Check carefully! Initial conditions are empty.
    ego_spec_orginal.init = set()
    test_spec_orginal.init = set()
    # Prog_guarantee and assumption are getting overwritten
    # Modify here!
    # for k in Vij_dict.keys():
    assumption = set()
    prog_guarantee = set()
    jmax = len(Vij) - 1
    Wij = dict()
    for j in np.linspace(jmax, 0, jmax+1):
        if j%2 == 0:
            # Vij = Vij_dict
            # st()
            test_spec = deepcopy(test_spec_orginal)
            ego_spec = deepcopy(ego_spec_orginal)
            assumption, prog_guarantee, goal_states = add_psi_i_j_progress(Vij, j, ver2st_dict, state_tracker)
            test_spec.prog |= prog_guarantee
            ego_spec.safety |= assumption

            # print(gr_spec.pretty())
            # test_spec.prog |= prog_guarantee
            # ego_spec.safety |= assumption
            gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to

            w_set = WinningSet()
            w_set.set_spec(gr_spec)

            aut = w_set.make_compatible_automaton(gr_spec)
            # g = synthesize_some_controller(aut)
            agentlist = ['x1', 'x2']
            fp = w_set.find_winning_set(aut)
            # print("Printing states in fixpoint: ")
            # pdb.set_trace()
            states_in_fp, states_out_fp = check_all_states_in_fp(tracklength, agentlist, w_set, fp, aut)
            if PRINT_STATES_IN_COMPUTATION:
                print(" ")
                print("Printing states in winning set: ")
            # Filter out states that begin in Vj, are in the fixpoint, and satisfy the assumptions
            start_set = Vij[j]

            states_in_winset, states_out_winset = check_all_states_in_winset_rh(tracklength, agentlist, w_set, fp, aut, merge_setting, state_test_dict, state_system_dict, goal_states, G, ver2st_dict, start_set)
            # st()
            Wij.update({j: states_in_winset})
    # st()
    return Wij

def get_tester_states_in_winsets(tracklength, merge_setting):
    """
    Find all tester states in the winning sets.
    W - Winning set
    indices:
    i - index of the goal
    j - distance to corresponding goal in steps
    k - index of the distance to the goal in j-1
    """
    ego_spec, test_spec, Vij_dict, state_tracker, ver2st_dict, G, state_dict_test, state_dict_system = specs_car_rh(tracklength, merge_setting)
    Wij = dict()
    # Go through all the goal states
    # st()
    for key in Vij_dict.keys():
        Wj = get_winset_rh(tracklength, merge_setting, Vij_dict[key], state_tracker, ver2st_dict,ego_spec, test_spec, state_dict_test, state_dict_system, G)
        Wij.update({key: Wj})
    # st()
    # Now find all the ks in each Wij

    # For each i, for each j in Wij - we can find the k (number to steps to j-1)
    # Wijk_vertex = deepcopy(Wijs)
    """
    Wijks = dict()
    for i in Wij.keys():
        Wjs = dict()
        for j in Wij[i].keys():
            # Find intermediate goal state for this j, which is j-2
            if j > 2:
                goals = Vij_dict[i][j-2]
            else:
                goals = Vij_dict[i][0]
            # add the temporary goal state to the graph for shortest path later
            new_G = deepcopy(G)
            new_G.add_node('temp_goal')
            for goal in goals:
                new_G.add_edge(goal, 'temp_goal')
            # Now go through all states in this Wj and order for k
            Wk_dict_inv = dict()
            for state in Wij[i][j]:
                # find vertex number of the states
                state_list = get_dict_inv_multiple(ver2st_dict, state)
                for ver in state_list:
                    if state_tracker[ver] == 't': # only tester states
                        state_vertex = ver
                # find shortest path to the goal states in Vij-2
                try:
                    k = shortest_path_length(new_G, state_vertex, 'temp_goal')-1
                except:
                    st()
                Wk_dict_inv.update({state_vertex: k})
            # Now create the dictionary that maps to each k the list of states
            Wks = dict()
            unique_ks = set(Wk_dict_inv.values())
            for k in unique_ks:
                Wk = list()
                for state in Wk_dict_inv.keys():
                    if Wk_dict_inv[state] == k:
                        Wk.append(state)
                Wks.update({k: Wk})
            Wjs.update({j: Wks})
            Wijks.update({i: Wjs})
    """
    return Wij, Vij_dict, state_tracker, ver2st_dict

def check_if_state_in_winset(state):
    pass


def check_system_states_in_winset(origin_state, state, ver2st_dict, state_tracker, Wij, debug = False):
    # st()
    if origin_state == {'x': 4, 'y': 1, 'x1': 5, 'y1': 2, 'x2': 3, 'y2': 2}:
        st()
    next_states = find_next_states(state, ver2st_dict, state_tracker)

    # Find the next states in vertex represenation - probably not necessary
    next_states_ver = []
    for state in next_states:
        vertices = get_dict_inv_multiple(ver2st_dict, state)
        for vertex in vertices:
            if state_tracker[vertex] == 't':
                next_states_ver.append(vertex)

    # Find if all of these states are in Wij
    # st()
    # Find j or original state for each i and compare to that value for all new states
    progress = False
    for i in Wij:
        if debug:
            st()
        j_next = dict()
        num_state = dict()
        for statenum,next_state in enumerate(next_states):
            num_state.update({statenum: next_state})
            j_next.update({statenum: None})
        j_original = None
        for j in Wij[i]:
            if debug:
                st()
            # check if state is in the winning set
            if origin_state in Wij[i][j]:
                j_original = j
            for statenum, next_state in enumerate(next_states):
                if debug:
                    st()
                if next_state in Wij[i][j]:
                    storej = j
                    if j == 0.0:
                        storej = 'goal'
                    j_next.update({statenum: storej})
        # st()
        if j_original and all(j_next.values()):
            for key in j_next.keys():
                if j_next[key] == 'goal':
                    j_next[key] = 0
            j_next_max = max(j_next, key=j_next.get)
            if j_next_max <= j_original:
                progress = True
    return progress

def find_next_states(state, ver2st_dict, state_tracker):
    """
    Propagate forward to check if the system state is in the winning set.
    """
    # find sys state in dict form if given as vertex
    if not isinstance(state,dict):
        state = ver2st_dict[state]
    x = state['x']
    y = state['y']
    tester_states = [(state['x1'],state['y1']),(state['x2'],state['y2'])]
    # feed in system state and simulate all possible next states
    # st()
    next_states = []
    actions =  {'move': (1,0), 'stay': (0,0), 'merge': (1,1)}
    # check for merge
    move_x, move_y = actions['merge']
    act_x = x + move_x
    act_y = y + move_y
    if (act_x,act_y) not in tester_states:
        # return the next state as merged
        state_dict_form = make_dict_form((act_x,act_y), tester_states)
        next_states.append(state_dict_form)
        return next_states
    else:
        for action in actions.keys():
            move_x, move_y = actions[action]
            act_x = x + move_x
            act_y = y + move_y
            if (act_x,act_y) not in tester_states:
                state_dict_form = make_dict_form((act_x,act_y), tester_states)
                next_states.append(state_dict_form)
        return next_states

def make_dict_form(sys_state, tester_states):
    state_dict = {'x': sys_state[0], 'y': sys_state[1], 'x1': tester_states[0][0], 'y1':tester_states[0][1], 'x2': tester_states[1][0], 'y2': tester_states[1][1]}
    return state_dict

def get_dict_inv_multiple(dict_in, value):
    keys = []
    for k, v in dict_in.items():
        if v == value:
            keys.append(k)
    return keys

if __name__ == '__main__':
    goal_loc = (3,0)
    intersectionfile = 'intersectionfile.txt'
    map, crosswalk = create_intersection_from_file(intersectionfile)
    G, sys2vertex, test2vertex = get_graph(map, crosswalk)
