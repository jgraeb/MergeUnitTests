import tulip as tulip
import numpy as np
from tulip.spec import form
# from scene import Scene
# from agent import Agent
# from map import Map
import networkx as nx
from omega.symbolic import temporal as trl
import _pickle as pickle
import os
from omega.games import enumeration as enum
from omega.symbolic import enumeration as sym_enum
from copy import deepcopy
import pdb
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
from ipdb import set_trace as st
from tulip import transys, spec, synth
from correct_win_set import get_winset, WinningSet, make_grspec, check_all_states_in_fp, check_all_states_in_winset, check_all_states_in_winset_rh, Spec
import networkx as nx
from networkx.algorithms.shortest_paths.generic import shortest_path_length
from winset_constants import PRINT_STATES_IN_COMPUTATION


# Merge example
# Tracklength
# tracklength = 5

class Spec:
    def __init__(self,sys_vars,init,safety,progress):
        self.variables = sys_vars
        self.init = init
        self.safety = safety
        self.prog = progress

# Function to get transitions of states:
# Coordinate system, if an agent is in (x,y), they can transition to (x,y), (x+1, y) or (x+1, y+1)
def get_map_transition(s, f, agent):
    edges = dict()
    if agent == "ego":
        for yi in range(1,3):
            for xi in range(s, f):
                if yi == 1:
                    edges[(xi, yi)] = [(xi,yi), (xi+1, yi), (xi+1, yi+1)]
                else:
                    edges[(xi, yi)] = [(xi,yi), (xi+1, yi)]
            edges[(f, yi)] = [(f,yi)]

    else:
        for yi in range(2,3):
            for xi in range(s, f):
                edges[(xi, yi)] = [(xi,yi), (xi+1, yi)]
            edges[(f, yi)] = [(f,yi)]
    return edges

# Get agent transitions:
def get_agent_transitions(s, f, agent_vars):
    """
    Parameters
    ----------
    s : int
        Initial cell index where the track starts
    f : int
        Final cell index where the track ends
    agent_vars : List[str]
        List of the string variables defining the parameters for the agent.
        ex: ['x', 'y'] for the system, ['x1', 'y1'] for the first tester and
        ['x2', 'y2'] for the second tester

    Returns
    -------
    State transition dictionary
    """
    agent_transition_dict = dict()
    agent_vars_dict = dict()
    checked_st = []
    # pdb.set_trace()
    if agent_vars[0]=='x' and agent_vars[1]=='y':
        transitions = get_map_transition(s, f,"ego")
    else:
        transitions = get_map_transition(s, f,"tester")
    ki = 1
    for k, v in transitions.items():
        start_st = {agent_vars[0]: k[0], agent_vars[1]: k[1]}
        if start_st not in checked_st:
            agent_vars_dict.update({ki: start_st})
            checked_st.append(start_st)
        else:
            ki = list(agent_vars_dict.keys())[list(agent_vars_dict.values()).index(start_st)]
        agent_transition_dict[ki] = []
        for vi in v:
            next_st = {agent_vars[0]: vi[0], agent_vars[1]: vi[1]}
            agent_transition_dict[ki].append(next_st)
        ki += 1
    return agent_transition_dict, agent_vars_dict

# Checking if the values of two dictionaries are equal:
def dict_equal(dict1, dict2):
    flg = False
    l1 = len(dict1)
    l2 = len(dict2)
    agent_vars_dict1 = []
    agent_vals_dict1 = []
    agent_vars_dict2 = []
    agent_vals_dict2 = []
    for ii in range(int(l1/2)):
        xi = 2*ii - 2
        yi = 2*ii - 1
        agent_vars_dict1.append((list(dict1.keys())[xi], list(dict1.keys())[yi]))
        agent_vals_dict1.append((list(dict1.values())[xi], list(dict1.values())[yi]))

    for ii in range(int(l2/2)):
        xi = 2*ii - 2
        yi = 2*ii - 1
        agent_vars_dict2.append((list(dict2.keys())[xi], list(dict2.keys())[yi]))
        agent_vals_dict2.append((list(dict2.values())[xi], list(dict2.values())[yi]))

    common_idx = [ii for ii in range(len(agent_vals_dict1)) if agent_vals_dict1[ii] in agent_vals_dict2]
    for cidx in common_idx:
        dict2_cidx = [ii for ii in range(len(agent_vals_dict2)) if agent_vals_dict2[ii] == agent_vals_dict1[cidx]]
        for c2idx in dict2_cidx:
            # if l1 > 2 or l2 > 2:
            #     pdb.set_trace()
            if not (agent_vars_dict1[cidx] == agent_vars_dict2[c2idx]):
                flg = True
                return flg
    return flg

# Function to get crossproduct of transitions for different states:
def get_transitions_cross_product_concurrent(trans1_dict, trans1_vars, trans2_dict, trans2_vars):
    """
    Parameters
    ----------
    trans1_dict : Dictionary of 1st agent transitions
    trans2_dict :Dictionary of 2nd agent transitions
    prod_type : str, optional
        Indicating which type of crossproduct needs to be performed: turn-based
        (as in the case of system and env cross product) or concurrent
        (as in when taking the cross product of all tester variables).

    Returns
    -------
    A single transition dictionary
    """
    product_dict = dict()
    prod_key_n = 1
    product_vars = dict()
    checked_states = []
    for s1_n, f1_list in trans1_dict.items():
        for s2_n, f2_list in trans2_dict.items():
            s1 = trans1_vars[s1_n]
            s2 = trans2_vars[s2_n]
            if not dict_equal(s1, s2): # Don't start in a collision state
                prod_key = {**s1, **s2} # Combining the states
                if prod_key not in checked_states:
                    product_vars.update({prod_key_n: prod_key})
                    checked_states.append(prod_key)
                else:
                    prod_key_n = list(product_vars.keys())[list(product_vars.values()).index(prod_key)]
                product_dict[prod_key_n] = []
                for f1_i in f1_list:
                    for f2_i in f2_list:
                        prod_val = {**f1_i, **f2_i}
                        if not dict_equal(f1_i, f2_i): # Avoid collisions:
                            product_dict[prod_key_n].append(prod_val)

                prod_key_n += 1
    return product_dict, product_vars

# Get transitions cross-product fo turn-based settings:
def get_transitions_cross_product_turn_based(ego_dict, ego_vars, test_dict, test_vars):
    """
    Parameters
    ----------
    ego_dict : Dictionary of 1st agent (ego) transitions
    test_dict :Dictionary of 2nd agent (tester) transitions

    Returns
    -------
    Product transition dictionary split into two dictionaries; one for the ego player and one for the
    tester player, and the state_tracker dictionary
    """
    product_dict_ego = dict()
    product_dict_test = dict()
    prod_key_n = 1
    product_vars_ego = dict()
    product_vars_test = dict()
    checked_states_ego = []
    checked_states_test = []
    state_tracker = dict()
    for s1_n, f1_list in ego_dict.items():
        for s2_n, f2_list in test_dict.items():
            s1 = ego_vars[s1_n]
            s2 = test_vars[s2_n]
            if not dict_equal(s1, s2): # Don't start in a collision state
                prod_key = {**s1, **s2} # Combining the states
                product_vars_ego.update({prod_key_n: prod_key})
                checked_states_ego.append(prod_key)
                state_tracker[prod_key_n] = "s"
                product_dict_ego[prod_key_n] = []
                for f1_i in f1_list:
                    prod_val = {**f1_i, **s2}
                    if not dict_equal(f1_i, s2): # Avoid collisions:
                        product_dict_ego[prod_key_n].append(prod_val)
                prod_key_n += 1

    for s1_n, f1_list in ego_dict.items():
        for s2_n, f2_list in test_dict.items():
            s1 = ego_vars[s1_n]
            s2 = test_vars[s2_n]
            if not dict_equal(s1, s2): # Don't start in a collision state
                prod_key = {**s1, **s2} # Combining the states
                product_vars_test.update({prod_key_n: prod_key})
                checked_states_test.append(prod_key)
                state_tracker[prod_key_n] = "t"
                product_dict_test[prod_key_n] = []
                for f2_i in f2_list:
                    prod_val = {**f2_i, **s1}
                    if not dict_equal(f2_i, s1): # Avoid collisions:
                        product_dict_test[prod_key_n].append(prod_val)
                prod_key_n += 1
    return product_dict_ego, product_vars_ego, product_dict_test, product_vars_test, state_tracker

# Function to define all possible states in the graph:
def get_all_states(tracklength, ego_n, tester_n):
    """
    Parameters
    ----------
    tracklength : int
        DESCRIPTION.
    ego_vars : range of ego variables
        DESCRIPTION.
    tester_vars : range of tester variables
        DESCRIPTION.

    Returns
    -------
    G : TYPE
        DESCRIPTION.
    st2ver_dict : TYPE
        DESCRIPTION.
    ver2st_dict : TYPE
        DESCRIPTION.
    """

    # nstates = 2 * len(ego_vars)
    # for ti_vars in tester_vars:
    #     nstates *= len(ti_vars)

    nstates = 2*ego_n*tester_n
    V = np.linspace(1, 1, nstates)
    G = nx.DiGraph()

    ego_T, ego_vars_T = get_agent_transitions(1, tracklength, ['x', 'y'])
    test1_T, test1_vars_T = get_agent_transitions(2, tracklength, ['x1', 'y1'])
    test2_T, test2_vars_T = get_agent_transitions(1, tracklength-1, ['x2', 'y2'])

    tester_T, tester_vars_T = get_transitions_cross_product_concurrent(test1_T, test1_vars_T, test2_T, test2_vars_T)
    states_ST_ego, states_vars_ST_ego, states_ST_test, states_vars_ST_test, state_tracker = get_transitions_cross_product_turn_based(ego_T, ego_vars_T, tester_T, tester_vars_T)

    # pdb.set_trace()
    st2ver_dict = dict()
    ver2st_dict = dict()
    for node in range(len(states_ST_ego.keys())):
        si = list(states_vars_ST_ego.keys())[node]
        st2ver_dict.update({si: node+1})
        ver2st_dict.update({node+1: states_vars_ST_ego[si]})
        G.add_node(node+1)

    for node in range(len(states_ST_ego.keys()), len(states_ST_ego.keys()) + len(states_ST_test.keys())):
        si = list(states_vars_ST_test.keys())[node - len(states_ST_ego.keys())]
        st2ver_dict.update({si: node+1})
        assert(si  == node+1)
        ver2st_dict.update({node+1: states_vars_ST_test[si]})
        G.add_node(node+1)

    for si, val_i in states_ST_ego.items():
        for fi in val_i:
            si_node = st2ver_dict[si]
            fi_node = get_dict_inv(states_vars_ST_test, fi)
            G.add_edge(si_node, fi_node)

    for si, val_i in states_ST_test.items():
        for fi in val_i:
            si_node = st2ver_dict[si]
            fi_node = get_dict_inv(states_vars_ST_ego, fi)
            G.add_edge(si_node, fi_node)

    return G, st2ver_dict, ver2st_dict, state_tracker, states_vars_ST_test, states_vars_ST_ego

# Function to get inverse mapping of a dictionary
# whose values are a dictionary
def get_dict_inv(dict_in, value):
    key = ''
    for k, v in dict_in.items():
        if v == value:
            key = k
    return key

# Add auxiliary nodes:
def add_aux_nodes(G, ver2st_dict, goal_state_cond):
    """
    Parameters
    ----------
    G : networkx.DiGraph()
        DESCRIPTION.
    goal_state_cond : lambda function that checks if a state is in the winning set
        lambda function

    Returns
    -------
    G_aux (graph), and target (node)

    """
    G_aux = G.deepcopy()
    target = "goal"
    for vi in G.nodes():
        vi_st = ver2st_dict[vi]
        if goal_state_cond(vi_st):
            G_aux.add_edge(vi, target)
    return G_aux, target

# Get all states that satisfy the goal condition:
def get_goal_states(G, goal_lambda, ver2st_dict, state_tracker):
    """
    Parameters
    ----------
    G : Graph
        Graph connecting all nodes of the graph
    goal_lambda : Lambda function description
        Used to check which satisfy the lambda function.

    Returns
    -------
    goal_states : List[dict[str]]
        List of states that satisfy the goal condition
        The states are given as a dictionary

    """
    goal_nodes = []
    for k in list(G.nodes):
        # pdb.set_trace()
        if state_tracker[k] == "t":
            k_st = ver2st_dict[k]
            if goal_lambda(k_st):
                goal_nodes.append(k)
    return goal_nodes

# Function to construct the lambda function for winning condition for the merge
# example:
def construct_lambda_function(merge_setting):
    if merge_setting == "between":
        goal_lambda = lambda st: (st['y'] == 2 and st['y2'] == 2 and st['y1'] == 2) and (st['x'] == st['x1']-1 and st['x'] == st['x2']+1)
    elif merge_setting == "front":
        goal_lambda = lambda st: (st['y'] == 2 and st['y2'] == 2 and st['y1'] == 2) and (st['x'] == st['x1']+1 or st['x'] == st['x2']+1)
    elif merge_setting == "back":
        goal_lambda = lambda st: (st['y'] == 2 and st['y2'] == 2 and st['y1'] == 2) and (st['x'] == st['x1']-1 or st['x'] == st['x2']-1)
    else:
        print("Merge setting incorrect")
        assert merge_setting in ["between", "front", "back"]
    return goal_lambda

# Function to collect Wj for each goal:
# Depending on the goal, there is a different ordering of Wj's of for each goal i.
# This function returns a dictionary of the set of Wjs for each goal
# Intuition: This is similar to specifying the terminal condition in MPC
# Depending on states that satisfy the goal_lambda condition, those are added to the Wj_dict
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
        Vj_dict[i] = get_Wj(tracklength, G, st2ver_dict, ver2st_dict, i)
    return Vj_dict

# Function to get Wi_j, which is the set of all states that are j*horizon steps
# (1 step = 1 round of play) away from set from progress goal []<>i:
def get_Wj(tracklength, G, st2ver_dict, ver2st_dict, goal):
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
    Wj_set : TYPE
        DESCRIPTION.

    """
    # G_aux, target = add_aux_nodes(G, goal_state)
    node_dist = dict()
    Wj_set = dict()
    for node in list(G.nodes):
        if nx.has_path(G, node, goal):
            pl = shortest_path_length(G, node, goal)
            node_dist[node] = pl
            if pl not in list(Wj_set.keys()):
                Wj_set[pl] = [node]
            else:
                Wj_set[pl].append(node)
    return Wj_set


# Ego safe controller for merging in between for specific tracklength and merge_setting:
def get_ego_safety(tracklength, merge_setting):
    ego_safe = set()

    # Dynamics for merging into adjacent track:
    for ii in range(1,tracklength):
        ego_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=2))'}
        ego_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2)|| (x='+str(ii+1)+' && y=1))'}
    ego_safe |= {'(x='+str(tracklength)+' && y=2 )-> X(x='+str(tracklength)+' && y=2)'}
    ego_safe |= {'(x='+str(tracklength)+' && y=1 )-> X(x='+str(tracklength)+' && y=1)'}

    # If there is no env car in the kitty-corner lane, the car merges:
    for xi in range(1, tracklength-1):
        merge_if_possible = '(!(y1=2 && x1 = '+str(xi+1) +') && !(y2=2 && x2 = '+str(xi+1) +')) -> X(y=2 && x='+str(xi+1)+')'
        ego_safe |= {merge_if_possible}

    # Last cell merge:
    merge_if_possible = '(!(y1=2 && x1 = '+str(tracklength) +')) -> X(y=2 && x='+str(tracklength)+')'
    ego_safe |= {merge_if_possible}

    # Don't start until x1 is in front:
    ego_safe |= {'(x1=2 && y1=2) -> X(x=1 && y=1)'}
    return ego_safe

# Function to add progress properties of the jth specification for the ith goal:
# Keep separate goals for each winning set
# i: goal
# Vij_dict: Dictionary
# j: Distance from the goal
def add_psi_i_j_progress(Vij_dict, j, ver2st_dict, state_tracker):
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

# Define string for state specification:
def get_str_spec(state_dict):
    spec_state = ""
    for k, v in state_dict.items():
        if spec_state == "":
            spec_state += "(" + k + " = " + str(v)
        else:
            spec_state += " && " + k + " = " + str(v)
    spec_state += ")"
    return spec_state

# Function to construct set membership:
def construct_spec_set_membership(Vj, ver2st_dict):
    spec = "("
    for vj in Vj:
        state_dict = ver2st_dict[vj]
        if spec == "(":
            spec += get_str_spec(state_dict)
        else:
            spec += " || " + get_str_spec(state_dict)
    spec += ")"
    return spec

# Safety specifications for the test agent:
def get_test_safety(tracklength, merge_setting):
    '''
    Parameters
    ----------
    tracklength : int
        Length of the road.
    merge_setting : str
        set to one of: between, front or back.

    Returns
    -------
    test_safe : set(str)
        A set of safety formulae that the testers need to follow.
        This function returns the dynamics of the test agents

    '''
    test_safe = set()
    for ii in range(2,tracklength-1):
        test_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1)|| (x1='+str(ii+1)+' && y1=2))'}
        test_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2)|| (x2='+str(ii+1)+' && y2=1))'}
        test_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1)|| (x2='+str(ii+1)+' && y2=2))'}
        test_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2)|| (x1='+str(ii+1)+' && y1=1))'}
    test_safe |= {'(x2='+str(tracklength)+' && y2=2) -> X(x2='+str(tracklength)+' && y2=2)'}
    test_safe |= {'(x2='+str(tracklength)+' && y2=1) -> X(x2='+str(tracklength)+' && y2=1)'}

    test_safe |= {'(x2=1 && y2=2) -> X((x2=1 && y2=2) || (x2=2 && y2=2) || (x2=2 && y2=1))'}
    test_safe |= {'(x2=1 && y2=1) -> X((x2=1 && y2=1) || (x2=2 && y2=2) || (x2=2 && y2=1))'}

    test_safe |= {'(x1='+str(tracklength)+' && y1=2) -> X(x1='+str(tracklength)+' && y1=2)'}
    test_safe |= {'(x1='+str(tracklength)+' && y1=1) -> X(x1='+str(tracklength)+' && y1=1)'}

    test_safe |= {'(x1='+str(tracklength-1)+' && y1=1) -> X((x1='+str(tracklength-1)+' && y1=1)|| (x1='+str(tracklength)+' && y1=2) || (x1='+str(tracklength)+' && y1=1)) '}
    test_safe |= {'(x1='+str(tracklength-1)+' && y1=2) -> X((x1='+str(tracklength-1)+' && y1=2)|| (x1='+str(tracklength)+' && y1=1) || (x1='+str(tracklength)+' && y1=2))'}
    return test_safe

# Function to add merge specifications:
def add_merge_specs(tracklength, merge_setting, test_safe):
    """
    Parameters
    ----------
    tracklength : TYPE
        DESCRIPTION.
    merge_setting : TYPE
        DESCRIPTION.
    test_safe : TYPE
        DESCRIPTION.

    Returns
    -------
    test_safe : TYPE
        DESCRIPTION.

    """
    merge_spec = '((x=2 && y=2) -> (x1=3 && x2=1 && y1=2 && y2=2))'
    for ki in range(2,tracklength-1):
        merge_spec = merge_spec + ' || ((x='+str(ki+1)+' && y=2) -> ((x1='+str(ki+2)+' && (x2='+str(ki)+') && y1=2 && y2=2)))'
    test_safe |= {merge_spec}
    return test_safe

# Function to get all possible states in the graph of all system and environment transitions:
def specs_car_rh(tracklength, merge_setting):
    # tracklength = 10
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
            st()
            Wij.update({j: states_in_winset})
    # st()
    return Wij

## ToDo:
# Add function to return viable tester states

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
    Wijs = dict()
    # Go through all the goal states
    # st()
    for key in Vij_dict.keys():
        Wj = get_winset_rh(tracklength, merge_setting, Vij_dict[key], state_tracker, ver2st_dict,ego_spec, test_spec, state_dict_test, state_dict_system, G)
        Wijs.update({key: Wj})
    # st()
    # Now find all the ks in each Wij

    # For each i, for each j in Wij - we can find the k (number to steps to j-1)
    # Wijk_vertex = deepcopy(Wijs)
    Wijks = dict()
    for i in Wijs.keys():
        Wjs = dict()
        for j in Wijs[i].keys():
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
            for state in Wijs[i][j]:
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
    st()

    st()
    state = 1
    in_ws = check_system_states_in_winset(state, ver2st_dict, state_tracker, Wijk)

    st()

def check_if_state_in_winset(state):
    pass


def check_system_states_in_winset(state, ver2st_dict, state_tracker, Wijk):
    next_states = find_next_states(state, ver2st_dict, state_tracker)
    next_states_ver = []
    for state in next_states:
        vertices = get_dict_inv_multiple(ver2st_dict, state)
        for vertex in vertices:
            if state_tracker[vertex] == 't':
                next_states_ver.append(vertex)
    # Find if all of these states are in Wijk
    st()
    for W_i in Wijk:
        for W_ij in W_i:
            for W_ijk in W_ij:
                # check if state is in the winning set
                for next_state in next_states_ver:
                    if next_state not in W_ijk:
                        return False
    return True



def find_next_states(ver_state, ver2st_dict, state_tracker):
    """
    Propagate forward to check if the system state is in the winning set.
    """
    print("checking if state in winset")
    # st()
    # find sys state in dict form
    state = ver2st_dict[ver_state]
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
    tracklength = 4
    merge_setting = "between"
    # state_tracker: keeps track of all system and tester states
    # ego_spec, test_spec, Vij_dict, state_tracker, ver2st_dict = specs_car_rh(tracklength, merge_setting)
    # # pdb.set_trace()
    # # states_in_winset returns only tester states in winset
    # # Modify here: Add function to simulate forward and get system states that only lead into the next winning set
    # states_in_winset = get_winset_rh(tracklength, merge_setting, Vij_dict, state_tracker, ver2st_dict)
    get_tester_states_in_winsets(tracklength, merge_setting)
