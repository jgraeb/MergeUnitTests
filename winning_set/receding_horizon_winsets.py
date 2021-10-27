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

# Function to define all possible states in the graph:
def get_all_states(ego_vars, tester_vars):
    nstates = 2 * len(ego_vars)
    for ti_vars in tester_vars:
        nstates *= len(ti_vars)
    V = np.linspace(1, 1, nstates)
    G = nx.graph((V,E))
    return G, V, E, st2ver_dict
    
# Function to get Wi_j, which is the set of all states that are j*horizon steps (1 step = 1 round of play) away from set from progress goal []<>i:
def get_Wj(tracklength, goal_state, merge_setting, horizon):
    

# Function to get all possible states in the graph of all system and environment transitions:
def specs_car_rh(tracklength, merge_setting, timestep, horizon):
    
    return ego_spec, test_spec


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
