#############################################################################
#                                                                           #
# Basic class for grid world traffic simulation                             #
# Josefine Graebener, Apurva Badithela                                      #
# Caltech, March 2021                                                       #
#                                                                           #
#############################################################################
from random import choice
from mcts import MCTS, Node
import numpy as np
from scene import Scene
from agent import Agent
from map import Map
import os
from copy import deepcopy
from ipdb import set_trace as st
from helper import *
from gridworld import GridWorld

def new_init_scene():
    '''Setting up the initial scene as list of agents'''
    agents = []
    ego_tuple = Agent(name ="system", x = 1, y = 1, v=1, goal = 2)
    agents.append(ego_tuple)
    tester_tuple = Agent(name ="tester_0", x = 2, y = 2, v=1, goal = 2)
    agents.append(tester_tuple)
    tester_tuple_2 = Agent(name ="tester_1", x = 1, y = 2, v=1, goal = 2)
    agents.append(tester_tuple_2)
    # tester_tuple2 = Agent(name ="ag_env_1", x = 1, y = 2, v=1, goal = 2)
    # agents.append(tester_tuple2)
    return agents

def new_World():
    '''Create the gridworld from the initial scene'''
    init_scene = new_init_scene()
    return GridWorld(2, 10, init_scene)

def run_random_sim(maxstep):
    '''Run a random simulation / for debugging - not used in MCTS!!!'''
    # run a game
    output_dir = os.getcwd()+'/saved_traces/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'sim_trace.p'
    filepath = output_dir + filename
    gridworld = new_World()
    gridworld.setup_world()
    gridworld.save_scene()
    acts = ['mergeL','stay','move', 'mergeR']
    for i in range(0,maxstep):
        #gridworld.timestep = i
        print('Step {}'.format(i))
        # environment takes step
        for agent in gridworld.env_agents:
            gridworld.env_take_step(agent,np.random.choice(acts))
        gridworld.save_scene()
        # ego takes step
        gridworld.ego_take_step()
        # save the scene
        gridworld.save_scene()
        # check if we are done
        for agent in gridworld.ego_agents:
            if gridworld.check_terminal(agent):
                print('Goal reached')
                # save the trace
                print(gridworld.trace)
                #import pdb; pdb.set_trace()
                gridworld.save_trace(filepath)
                return
    self.save_trace(filepath)


def append_trace(trace_dict, agent):
    '''Append the trace'''
    trace_dict["x"].append(agent.x)
    trace_dict["y"].append(agent.y)
    trace_dict["v"].append(agent.v)


def play_game():
    '''Play the game using MCTS to find the strategy'''
    trace=[]
    tree = MCTS()
    # save the trace
    output_dir = os.getcwd()+'/saved_traces/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'sim_trace.p'
    filepath = output_dir + filename
    gridworld = new_World()
    gridworld.setup_world()
    trace = save_scene(gridworld,trace) # save initial scene
    acts = ['mergeL','stay','move', 'mergeR']
    ego_trace = {"x": [], "y": [], "v": []}
    env_trace = {"x": [], "y": [], "v": []}
    for agent in gridworld.env_agents:
        append_trace(env_trace, agent)
    for agent in gridworld.ego_agents:
        append_trace(ego_trace, agent)

    game_trace = [] # Same as ego_trace and env_trace condensed into one step with env going first
    k = 0 #  Time stamp
    # Initial step by environment:
    for agent in gridworld.env_agents:
        gridworld.env_take_step(agent,'move')
    for agent in gridworld.env_agents:
        append_trace(env_trace, agent)
        # trace = (gridworld,trace) # save first env action
    gridworld.print_state()
    while True:
        gridworld.ego_take_input('mergeR')  # Ego action
        for agent in gridworld.ego_agents:
            append_trace(ego_trace, agent)
        game_trace.append(deepcopy(gridworld))
        grid_term = gridworld.is_terminal()
        trace = save_scene(gridworld,trace)
        gridworld.print_state()
        if grid_term:
            if k==0:
                print("Poor initial choices; no MCTS rollouts yet")
            for agent in gridworld.ego_agents:
                if gridworld.width == agent.x and agent.y == 1:
                    print('Did not merge; end of road')
            else:
                print("Goal reached; ego successfully merged!")
            break
        else:
            k = k+1
        gridworldnew = deepcopy(gridworld)
        for k in range(500):
            #print("Rollout: ", str(k+1))
            tree.do_rollout(gridworldnew)
        gridworldnew = tree.choose(gridworldnew) # Env action
        # find which actions were chosen and take them
        actions = which_action(gridworld,gridworldnew)
        # st()
        for agent,action in zip(gridworld.env_agents,actions):
            gridworld.env_take_step(agent,action)
        for agent in gridworld.env_agents:
            append_trace(env_trace, agent)
        trace = save_scene(gridworld,trace)
        gridworld.print_state()
        grid_term = gridworld.is_terminal()
    save_trace(filepath,trace)
    return ego_trace, env_trace, game_trace

def which_action(gridworld,gridworldnew):
    '''Find from chosen gridworld which action was chosen for each agent'''
    actions = []
    for agentold in gridworld.env_agents:
        for agentnew in gridworldnew.env_agents:
            if agentold.name == agentnew.name:
                newx = agentnew.x
                oldx = agentold.x
                newy = agentnew.y
                oldy = agentold.y
                if newx == oldx:
                    action = 'stay'
                else:
                    action = 'move'
        actions.append(action)
    return actions

# Constructing trace:
def append_trace(trace_dict, agent):
    trace_dict["x"].append(agent.x)
    trace_dict["y"].append(agent.y)
    trace_dict["v"].append(agent.v)

if __name__ == '__main__':
    #run_random_sim(10)
    output_dir = os.getcwd()+'/saved_traces/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'sim_trace.p'
    filepath = output_dir + filename
    ego_trace, env_trace, game_trace = play_game()
    print("Ego trajectory")
    print(ego_trace)
    print("")
    print("Environment trajectory")
    print(env_trace)
