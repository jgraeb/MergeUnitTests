import tulip as tulip
import numpy as np
from tulip.spec import form
from scene import Scene
from agent import Agent
from map import Map
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
from correct_win_set import get_winset, WinningSet, make_grspec, check_all_states_in_fp, check_all_states_in_winset, Spec
import networkx as nx

# Merge example
# Tracklength
tracklength = 5 

# Function to get transitions of states:
# Coordinate system, if an agent is in (x,y), they can transition to (x,y), (x+1, y) or (x+1, y+1)
def get_map_transition(s, f):
    edges = dict()
    for yi in range(1,3):
        for xi in range(s, f):
            edges[(xi, yi)] = [(xi,yi), (xi+1, yi), (xi+1, yi+1)]
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
    transitions = get_map_transitions(s, f)
    for k, v in transitions.items():
        start_st = {agent_vars[0]: k[0], agent_vars[1]: k[1]}
        for vi in v:
            next_st = {agent_vars[0]: vi[0], agent_vars[1]: vi[1]}
            agent_transition_dict[start_st] = next_st
    return agent_transition_dict

# Function to get crossproduct of transitions for different states:
def get_transitions_cross_product(trans1_dict, trans2_dict, prod_type="concurrent"):
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
    for s1, f1_list in trans1_dict.items():
        for s2, f2_list in trans2_dict.items():
            prod_key = {**s1, **s2} # Combining the states
            product_dict[prod_key] = []
            if prod_type == "concurrent":
                for f1_i in f1_list:
                    for f2_i in f2_list:
                        prod_val = {**f1_i, **f2_i}
                        if f1_i != f2_i: # Avoid collisions:
                            product_dict[prod_key].append(prod_val)
                            
            elif prod_type == "turn-based":
                for f1_i in f1_list:
                    prod_val = {**f1_i, **s2}
                    if f1_i != s2: # Avoid collisions:
                        product_dict[prod_key].append(prod_val)
                
                for f2_i in f2_list:
                    prod_val = {**f2_i, **s1}
                    if f2_i != s1: # Avoid collisions:
                        product_dict[prod_key].append(prod_val)

            else:
                assert prod_type == "concurrent" or prod_type == "turn-based"
    return product_dict

# Function to define all possible states in the graph:
def get_all_states(tracklength, ego_vars, tester_vars):
    nstates = 2 * len(ego_vars)
    for ti_vars in tester_vars:
        nstates *= len(ti_vars)
    V = np.linspace(1, 1, nstates)
    G = nx.graph((V,E))
    
    ego_T = get_agent_transitions(1, tracklength, ["x", "y"])
    test1_T = get_agent_transitions(2, tracklength, ["x1", "y1"])
    test2_T = get_agent_transitions(1, tracklength-1, ["x2", "y2"])
    
    tester_T = get_transitions_cross_product(test1_T, test2_T, prod_type="concurrent")
    states_ST = get_transitions_cross_product(tester_T, ego_T, prod_type="turn-based")
    
    for node in range(len(states_ST.keys())):
        si = list(states_ST.keys())[node]
        st2ver_dict.update({node: si})
        ver2st_dict.update({si: node})
        V.add_node(node)
        
    for si, val_i in states_ST.items():
        for fi in val_i:
            si_node = ver2st_dict[si]
            fi_node = ver2st_dict[fi]
            G.add_edge(si_node, fi_node)
            
    return G, V, E, st2ver_dict


# Function to get Wi_j, which is the set of all states that are j*horizon steps 
# (1 step = 1 round of play) away from set from progress goal []<>i:
def get_Wj(tracklength, goal_state, merge_setting, horizon):
    
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

# Function to add progress properties of the jth specification:
def add_psi_j_progress(tracklength, j, test_safe, merge_setting):
    Wj = 
    return test_safe

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
    test_safe |= {'(x1='+str(tracklength-1)+' && y1=2) -> X((x1='+str(tracklength-1)+' && y1=2)|| (x1='+str(tracklength)+' && y1=1) || (x1='+str(tracklength)+' && y1=2))
    return test_safe

# Function to add merge specifications:
def add_merge_specs(tracklength, merge_setting, test_safe):
    merge_spec = '((x=2 && y=2) -> (x1=3 && x2=1 && y1=2 && y2=2))'
    for ki in range(2,tracklength-1):
        merge_spec = merge_spec + ' || ((x='+str(ki+1)+' && y=2) -> ((x1='+str(ki+2)+' && (x2='+str(ki)+') && y1=2 && y2=2)))'
    test_safe |= {merge_spec}
    return test_safe

# Function to get all possible states in the graph of all system and environment transitions:
def specs_car_rh(tracklength, merge_setting, timestep, horizon):
    ego_vars = set()
    ego_vars['x'] = (1, tracklength)
    ego_vars['y'] = (1,2)
    ego_init = {'x='+str(1), 'y='+str(1)}
    ego_safe = get_ego_safety(tracklength, merge_setting)
    ego_prog = {'y=2'}
    
    test_vars['x1'] = (1, tracklength)
    test_vars['x2'] = (1, tracklength)
    test_vars['y1'] = (1,2)
    test_vars['y2'] = (1,2)
    test_init = {'x1='+str(2), 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    test_safe = get_test_safety(tracklength, merge_setting)
    test_safe = add_merge_specs(tracklength, merge_setting, test_safe)
    test_prog = add_psi_j_progress(tracklength, j, merge_setting)
    
    return ego_spec, test_spec

# 

# Function to get all receding horizon winning sets:
# tracklength: length of the track; merge_setting: between/ in front/ behind
def get_winset_rh(tracklength, merge_setting, timestep, horizon):
    ego_spec, test_spec = specs_car_rh(tracklength, merge_setting, timestep, horizon) #spec_merge_in_front()#all_system(3)#spec_merge_in_front()#test_spec()#specs_for_entire_track(5)
    gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
    # print(gr_spec.pretty())
    w_set = WinningSet()
    w_set.set_spec(gr_spec)

    aut = w_set.make_compatible_automaton(gr_spec)
    # g = synthesize_some_controller(aut) 
    agentlist = ['x1', 'x2']
    fp = w_set.find_winning_set(aut)
    # print("Printing states in fixpoint: ")
    states_in_fp, states_out_fp = check_all_states_in_fp(tracklength, agentlist, w_set, fp, aut)
    print(" ")
    print("Printing states in winning set: ")
    states_in_winset, states_out_winset = check_all_states_in_winset(tracklength, agentlist, w_set, fp, aut, merge_setting)
    return states_in_winset
