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
    ego_snapshot = []
    env_snapshot = []
    for agent in gridworld.ego_agents:
        ego_snapshot.append((agent.name,agent.x,agent.y))
    for agent in gridworld.env_agents:
        env_snapshot.append((agent.name,agent.x,agent.y))
    current_scene = Scene(gridworld.timestep, gridworld.map, ego_snapshot, env_snapshot)
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
    filename = 'ws_out_files.pkl'
    filepath = output_dir + filename
    print('Saving winning set in pkl file')
    with open(filepath, 'wb') as pckl_file:
        pickle.dump(ws, pckl_file)

def load_ws():
    ws_file = os.getcwd()+'/highway_merge/saved_filters/ws_out_files.pkl'
    with open(ws_file, 'rb') as pckl_file:
        ws = pickle.load(pckl_file)
    Wij = ws['Wij']
    Vij_dict = ws['Vij_dict']
    state_tracker = ws['state_tracker']
    ver2st_dict = ws['ver2st_dict']
    return Wij, Vij_dict, state_tracker, ver2st_dict
