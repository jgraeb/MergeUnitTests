#############################################################################
#                                                                           #
# Helper functions for Compositional Testing gridworld simulation           #
# Josefine Graebener, Apurva Badithela                                      #
# Caltech, March 2021                                                       #
#                                                                           #
#############################################################################
from scene import Scene
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
