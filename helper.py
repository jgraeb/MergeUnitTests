#############################################################################
#                                                                           #
# Helper functions for Compositional Testing gridworld simulation           #
# Josefine Graebener, Apurva Badithela                                      #
# Caltech, March 2021                                                       #
#                                                                           #
#############################################################################
import sys
sys.path.append('..')
from components.scene import Scene
import _pickle as pickle
import os
from ipdb import set_trace as st


def save_trace(filename,trace): # save the trace in pickle file for animation
    print('Saving trace in pkl file')
    with open(filename, 'wb') as pckl_file:
        pickle.dump(trace, pckl_file)

def save_scene(gridworld,trace): # save each scene in trace
    print('Saving scene {}'.format(gridworld.timestep))
    sys_snapshot = []
    tester_snapshot = []
    ped_snapshot = []
    for agent in gridworld.sys_agents:
        sys_snapshot.append((agent.name,agent.x,agent.y, agent.orientation))
    for agent in gridworld.tester_cars:
        tester_snapshot.append((agent.name,agent.x,agent.y, agent.orientation))
    for ped in gridworld.tester_peds:
        ped_snapshot.append((ped.name, ped.x,ped.y, ped.cwloc))
    current_scene = Scene(gridworld.timestep, gridworld.map, sys_snapshot, tester_snapshot, ped_snapshot)
    trace.append(current_scene)
    gridworld.timestep += 1
    gridworld.trace = trace
    return trace

def save_ws_comp_result(Wij, Vij_dict, state_tracker, ver2st_dict):
    # save objects in dictionary
    ws = dict()
    ws.update({'Wij': Wij})
    ws.update({'Vij_dict': Vij_dict})
    ws.update({'state_tracker': state_tracker})
    ws.update({'ver2st_dict': ver2st_dict})
    # save dict in pkl file
    output_dir = os.getcwd()+'/highway_merge/saved_filters/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'ws_out_files.p'
    filepath = output_dir + filename
    print('Saving winning set in pkl file')
    with open(filepath, 'wb') as pckl_file:
        pickle.dump(ws, pckl_file)

def load_ws():
    ws_file = os.getcwd()+'/highway_merge/saved_filters/ws_out_files.p'
    with open(ws_file, 'rb') as pckl_file:
        ws = pickle.load(pckl_file)
    Wij = ws['Wij']
    Vij_dict = ws['Vij_dict']
    state_tracker = ws['state_tracker']
    ver2st_dict = ws['ver2st_dict']
    return Wij, Vij_dict, state_tracker, ver2st_dict

def save_ws_comp_result_intersection(Wij, Vij, G_aux, sys_st2ver_dict, test_st2ver_dict):
    # save objects in dictionary
    ws = dict()
    ws.update({'Wij': Wij})
    ws.update({'Vij': Vij})
    ws.update({'sys_st2ver_dict': sys_st2ver_dict})
    ws.update({'test_st2ver_dict': test_st2ver_dict})
    ws.update({'G_aux': G_aux})
    # save dict in pkl file
    output_dir = os.getcwd()+'/intersection/saved_filters/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'ws_out_files.p'
    filepath = output_dir + filename
    print('Saving winning set in pkl file')
    with open(filepath, 'wb') as pckl_file:
        pickle.dump(ws, pckl_file)

def load_ws_intersection():
    ws_file = os.getcwd()+'/intersection/saved_filters/ws_out_files.p'
    with open(ws_file, 'rb') as pckl_file:
        ws = pickle.load(pckl_file)
    Wij = ws['Wij']
    Vij = ws['Vij']
    sys_st2ver_dict = ws['sys_st2ver_dict']
    test_st2ver_dict = ws['test_st2ver_dict']
    G_aux = ws['G_aux']
    return Wij, Vij, G_aux, sys_st2ver_dict, test_st2ver_dict

def load_graph_dicts():
    g_file = os.getcwd()+'/intersection/saved_graph/graph_out.p'
    with open(g_file, 'rb') as pckl_file:
        g = pickle.load(pckl_file)
    Vij = g['Vij']
    G_aux = g['G_aux']
    sys_state2vertex = g['sys_state2vertex']
    test_state2vertex = g['test_state2vertex']
    return G_aux, Vij, sys_state2vertex, test_state2vertex

def save_graph_and_dicts(G_aux, Vij, sys_state2vertex, test_state2vertex):
    # save objects in dictionary
    g = dict()
    g.update({'G_aux': G_aux})
    g.update({'Vij': Vij})
    g.update({'sys_state2vertex': sys_state2vertex})
    g.update({'test_state2vertex': test_state2vertex})
    # save dict in pkl file
    output_dir = os.getcwd()+'/intersection/saved_graph/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'graph_out.p'
    filepath = output_dir + filename
    print('Saving graph set in pkl file')
    with open(filepath, 'wb') as pckl_file:
        pickle.dump(g, pckl_file)
