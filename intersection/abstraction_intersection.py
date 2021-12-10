import sys
sys.path.append('..')
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
from collections import OrderedDict as od


# STATE_DICT, ACTIONS_DICT = make_state_dictionary_for_specification()
def create_intersection_from_file(intersectionfile):
    map = od()
    f = open(intersectionfile, 'r')
    lines = f.readlines()
    len_y = len(lines)
    for i,line in enumerate(lines):
        for j,item in enumerate(line):
            if item != '\n':
                map[i,j] = item
    # make dictionary that maps each crosswalk state to a grid cell
    # currenly manual -> TODO automatically from file
    crosswalk = dict()
    start_cw = 2
    end_cw = 6
    y = 2
    for i, num in enumerate(range(2*start_cw,2*(end_cw+1))):
        crosswalk.update({i: (int(np.floor(num/2)), y)})
    return map, crosswalk

def get_system_states(state_dict):
    z_min_grid = 0
    z_max_grid = 7
    y_min_grid = 0
    y_max_grid = 7
    sys_states = []
    for ii in range(z_min_grid,z_max_grid):
        for jj in range(y_min_grid,y_max_grid):
            if (ii,jj) in state_dict:
                sys_states.append((ii,jj))
    return sys_states

def get_tester_states(state_dict):
    z_min_grid = 0
    z_max_grid = 7
    y_min_grid = 0
    y_max_grid = 7
    tester_states = []
    for ii in range(z_min_grid,z_max_grid):
        for jj in range(y_min_grid,y_max_grid):
            if (ii,jj) in state_dict:
                tester_states.append((ii,jj))
    return tester_states

def take_crossproduct(state_dict, states):
    crossproduct_states = []
    while True:
        if len(states) == 2:
            for state1 in states[0]:
                for state2 in states[1]:
                    if type(state2) != 'list':
                        if state1 != state2:
                            crossproduct_states.append(((state1),(state2)))
                    elif state1 not in state2:
                            crossproduct_states.append(((state1),(state2)))
            return crossproduct_states
        else:
            states[1] = take_crossproduct(state_dict, states[1:])

def find_next_states(state_dict,state):
    next_states = [[],[]]
    for i in range(len(state)):
        if state[i] in state_dict:
            next_states[i] = state_dict[state[i]]
    next_state_combinations = []
    for j in next_states[0]:
        for k in next_states[1]:
            if k != j:
                next_state_combinations.append((j,k))
    return next_state_combinations

def find_next_tester_states(state,next_state_dict,crosswalk):
    # st()
    next_states = [[],[]]
    # Find the possible system actions
    if state[0] in next_state_dict:
        next_states = next_state_dict[state[0]]
    # put the combinations back together leaving the tester states untouched
    next_tester_combinations = []
    # st()
    for jj in next_states:
        # st()
        if jj != state[1] and jj != crosswalk[state[2]]:
                next_tester_combinations.append((jj,state[1],state[2]))
    # st()
    return next_tester_combinations

def find_next_sys_states(state_dict, next_state_dict, state, crosswalk):
    next_states = [[],[]]
    tester_state = state[1:]
    # st()
    next_car_states = []
    # Find the possible tester car actions
    if tester_state[0] in state_dict:
        next_car_states = next_state_dict[tester_state[0]]
    # Find the possible tester pedestrian actions
    next_ped_states = []
    next_ped_states.append(tester_state[1])
    if tester_state[1] != 0:
        next_ped_states.append(tester_state[1]-1)
    if tester_state[1] != 6:
        next_ped_states.append(tester_state[1]+1)

    # put the combinations back together leaving the tester states untouched
    next_sys_state_combinations = []
    for kk in next_car_states:
        for jj in next_ped_states:
            if kk != state[0] and kk != crosswalk[jj] and state[0] != crosswalk[jj]:
                next_sys_state_combinations.append((state[0], kk, jj))
    # st()
    return next_sys_state_combinations

# def find_goal_state(state2vertex, goal_loc):
#     goal_states = []
#     for state in state2vertex:
#         if state[0]==goal_loc:
#             goal_states.append(state2vertex[state])
#     return goal_states

def find_next_state_dict(state_dict):
    next_state_dict = dict()
    for state in state_dict:
        if state_dict[state] != '*':
            new_states = []
            new_states.append(state)
            if state_dict[state] == '←' and state[1] > 0:
                new_state = (state[0]-1, state[1])
                new_states.append(new_state)
            if state_dict[state] == '↓' and 7 > state[0]:
                new_state = (state[0], state[1]+1)
                new_states.append(new_state)
            if state_dict[state] == '↑' and state[0] > 0:
                new_state = (state[0], state[1]-1)
                new_states.append(new_state)
            if state_dict[state] == '+':
                if state[0] == 3 and state[1] == 3:
                    new_state = (state[0]-1, state[1])
                    new_states.append(new_state)
                    new_state = (state[0], state[1]+1)
                if state[0] == 4 and state[1] == 3:
                    new_state = (state[0]+1, state[1])
                    new_states.append(new_state)
                    new_state = (state[0], state[1]+1)
                if state[0] == 3 and state[1] == 4:
                    new_state = (state[0]-1, state[1])
                    new_states.append(new_state)
                    new_state = (state[0], state[1]-1)
                if state[0] == 4 and state[1] == 4:
                    new_state = (state[0]+1, state[1])
                    new_states.append(new_state)
                    new_state = (state[0], state[1]-1)
                new_states.append(new_state)
            next_state_dict.update({state: new_states})
    # st()
    return next_state_dict

def get_graph(state_dict, crosswalk):
    z_max_sys = 7
    z_min_sys = 3
    y_max_sys = 4
    y_min_sys = 0
    sys_states = []
    for ii in range(z_min_sys, z_max_sys+1):
        for jj in range(y_min_sys, y_max_sys+1):
            if (ii,jj) in state_dict:
                if state_dict[(ii,jj)] == '↑' or state_dict[(ii,jj)] == '←' or state_dict[(ii,jj)] == '+':
                    sys_states.append((ii,jj))
    z_max_test = 7
    z_min_test = 0
    y_test = 3
    tester_states = []
    for ii in range(z_min_test, z_max_test):
            if (ii,y_test) in state_dict:
                tester_states.append((ii,y_test))

    ped_cw_loc_min = 0
    ped_cw_loc_max = 7
    ped_states = []
    for ii in range(z_min_test, z_max_test):
            if (ii) in crosswalk:
                ped_states.append((ii))

    nodes = []
    for sys_state in sys_states:
        for tester_state in tester_states:
            for ped_state in ped_states:
                if sys_state != tester_states and crosswalk[ped_state] != sys_state and crosswalk[ped_state] != tester_states:
                    nodes.append(((sys_state), (tester_state), (ped_state)))

    nstates = len(nodes)
    G = nx.DiGraph()
    V = np.linspace(1, 2*nstates, 2*nstates)
    G.add_nodes_from(V)
    
    state2vertex = dict()
    sys_state2vertex = dict()
    test_state2vertex = dict()
    # Now loop through the states to match with the numbered graph nodes
    for i,state in enumerate(nodes):
        # vertex2state.update({i+1: state})
        # st()
        sys_state2vertex.update({state: i + 1})
        test_state2vertex.update({state: i + 1 + nstates})

    next_state_dict = find_next_state_dict(state_dict)

    # Now loop through the states and create the edges
    edge_dict = dict()
    for state in nodes:
        # For each state depending on if it is a tester or system state, find the next state
        next_sys_states = find_next_sys_states(state_dict,next_state_dict, state, crosswalk)
        next_test_states = find_next_tester_states(state, next_state_dict, crosswalk)
        st()
        next_vertices = []
        next_sys_vertices = []
        next_test_vertices = []
        for next_sys_state in next_sys_states:
            try:
                next_sys_vertices.append(sys_state2vertex[next_sys_state])
            except:
                pass
                # st()
                # next_sys_vertices.append("offmap")
        for next_test_state in next_test_states:
            try:
                next_test_vertices.append(test_state2vertex[next_test_state])
            except:
                pass
                # st()
                # next_test_vertices.append("offmap")
        # Make the flip from system state to tester state
        edge_dict.update({sys_state2vertex[state]: next_test_vertices})
        edge_dict.update({test_state2vertex[state]: next_sys_vertices})
    # initialize edges in networkx graph
    for key in edge_dict.keys():
        for item in edge_dict[key]:
            G.add_edge(key,item)
    st()
    return G, sys_state2vertex, test_state2vertex


# Function to define all possible states in the graph:
def get_all_states():
    state_dict, actions_dict = make_state_dictionary_for_specification()
    # to do get these from the file
    z_min_grid = 0
    z_max_grid = 7
    y_min_grid = 0
    y_max_grid = 7
    # create the list of states - system states and tester states and save a dictionary with the mapping from the number to the state
    sys_states = [get_system_states(state_dict)]
    tester_states = get_tester_states(state_dict)
    states = deepcopy(sys_states)
    states.append(tester_states)
    crossproduct_states = take_crossproduct(state_dict, states)
    # Now every state exists as a system state and a tester state ! so crossproduct_states * 2 !
    # Now create the graph
    nstates = len(crossproduct_states)
    G = nx.DiGraph()
    V = np.linspace(1, 1, 2 * nstates)
    G.add_nodes_from(V)
    G.add_node("offmap") # Node representing that the agents left the map (system and tester node - What do do if just one tester leaves the map??)
    # vertex2state = dict()
    state2vertex = dict()
    sys_state2vertex = dict()
    test_state2vertex = dict()
    # Now loop through the states to match with the numbered graph nodes
    for i,state in enumerate(crossproduct_states):
        # vertex2state.update({i+1: state})
        # st()
        sys_state2vertex.update({state: i + 1})
        test_state2vertex.update({state: i + 1 + nstates})
    # Now loop through the states and create the edges
    edge_dict = dict()
    # st()
    for state in crossproduct_states:
        # For each state depending on if it is a tester or system state, find the next state
        next_sys_states = find_next_sys_states(state_dict,state)
        # st()
        next_test_states = find_next_tester_states(state_dict,state)
        # st()
        next_vertices = []
        next_sys_vertices = []
        next_test_vertices = []
        for next_sys_state in next_sys_states:
            try:
                next_sys_vertices.append(sys_state2vertex[next_sys_state])
            except:
                next_sys_vertices.append("offmap")
        for next_test_state in next_test_states:
            try:
                next_test_vertices.append(test_state2vertex[next_test_state])
            except:
                next_test_vertices.append("offmap")
        # Make the flip from system state to tester state
        edge_dict.update({sys_state2vertex[state]: next_test_vertices})
        edge_dict.update({test_state2vertex[state]: next_sys_vertices})
    # initialize edges in networkx graph
    for key in edge_dict.keys():
        for item in edge_dict[key]:
            G.add_edge(key,item)
    # st()
    return G, sys_state2vertex, test_state2vertex

def add_edges_2_goalstate(G, state2vertex, goal_states):
    G.add_node('goal')
    for goal_state in goal_states:
        G.add_edge(goal_state, 'goal')
    return G

# Function to get Wi_j, which is the set of all states that are j*horizon steps (1 step = 1 round of play) away from set from progress goal []<>i:
def get_Wj(G):
    index_dict = dict()
    for node in G.nodes():
        try:
            print(nx.dijkstra_path(G, source = node, target='goal'))
            length = nx.dijkstra_path_length(G, source = node, target='goal')
            print(length)
            index_dict.update({node: length})
        except:
            print('No path for node '+str(node))
    max_index = max(index_dict.values())
    Wj = dict()
    for idx in range(max_index):
        w_idx = []
        for key in index_dict:
            if index_dict[key] == idx:
                w_idx.append(key)
        Wj.update({idx: w_idx})
    return Wj

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
    agentlist = ['z1', 'z2']
    fp = w_set.find_winning_set(aut)
    # print("Printing states in fixpoint: ")
    states_in_fp, states_out_fp = check_all_states_in_fp(tracklength, agentlist, w_set, fp, aut)
    print(" ")
    print("Printing states in winning set: ")
    states_in_winset, states_out_winset = check_all_states_in_winset(tracklength, agentlist, w_set, fp, aut, merge_setting)
    return states_in_winset

if __name__ == '__main__':
    goal_loc = (3,0)

    intersectionfile = 'intersectionfile.txt'
    map, crosswalk = create_intersection_from_file(intersectionfile)
    G, sys_state2vertex, test_state2vertex = get_graph(map, crosswalk)
    st()
    # Old:
    G, sys_state2vertex, test_state2vertex = get_all_states()
    # goal_sys_states = find_goal_state(sys_state2vertex, goal_loc)
    # goal_test_states = find_goal_state(test_state2vertex, goal_loc)
    # goal_states = goal_sys_states + goal_test_states
    # state2vertex = sys_state2vertex | test_state2vertex
    state2vertex = {**sys_state2vertex, **test_state2vertex}
    goal_states = find_goal_state(state2vertex, goal_loc)
    G = add_edges_2_goalstate(G, state2vertex, goal_states)
    Wj = get_Wj(G)
    st()
