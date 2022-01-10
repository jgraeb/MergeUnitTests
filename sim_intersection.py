#############################################################################
#                                                                           #
# Basic class for grid world traffic simulation  intersection               #
# Josefine Graebener, Apurva Badithela                                      #
# Caltech, December 2021                                                    #
#                                                                           #
#############################################################################
import sys
sys.path.append('..')
from random import choice
from tree_search.mcts import MCTS, Node
import numpy as np
from components.scene import Scene
from components.agent import Agent
from components.pedestrian import Pedestrian
from components.map import Map
import os
from copy import deepcopy
from ipdb import set_trace as st
from helper import *
from intersection.intersection import GridWorld
from highway_merge.test_parameters import TRACKLENGTH, MERGE_SETTING


def new_init_scene(sys_agent = None, list_of_agents = None):
    '''Setting up the initial scene as list of agents'''
    if sys_agent == None:
        agent = (7,4) # initial position of system under test
    if list_of_agents == None:
        list_of_agents = [('Agent1',0,3,'s', 's'), ('Pedestrian', 2, 2, 0, 's')]
    #
    sys_agents = [Agent(name='ego', x=agent[0],y=agent[1],v=0, goal='w', orientation = 'n')]
    tester_peds = []
    tester_cars = []
    for agent in list_of_agents:
        if agent[0] == 'Pedestrian':
            tester_peds.append(Pedestrian(name='Ped1', x=agent[1],y=agent[2],cwloc=agent[3], goal=agent[4]))
        else:
            tester_cars.append(Agent(name=agent[0], x=agent[1],y=agent[2],v=0, goal=agent[3], orientation = agent[4]))
    return sys_agents, tester_cars, tester_peds

def new_World(intersectionfile):
    '''Create the gridworld from the initial scene'''
    gridworld = GridWorld([],intersectionfile)
    # self.find_actions_for_cell()
    sys_agents, tester_cars, tester_peds = new_init_scene()
    # self.tester_agents = [Agent(name=item[0], x=item[1],y=item[2],v=0, goal=item[3], orientation = item[4]) for item in list_of_agents]
    # agents = self.sys_agents + self.tester_agents
    gridworld.sys_agents = sys_agents
    gridworld.tester_cars = tester_cars
    gridworld.tester_peds = tester_peds
    gridworld.print_intersection(gridworld.timestep)
    return gridworld

def run_random_sim(maxstep):
    '''Run a random simulation / for debugging - not used in MCTS!!!'''
    print('Intersection Example')
    intersectionfile = os.getcwd()+'/intersection/intersectionfile.txt'
    gridworld = new_World(intersectionfile)

    output_dir = os.getcwd()+'/intersection/saved_traces/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'sim_trace.p'
    filepath = output_dir + filename
    # gridworld = new_World()
    # gridworld.setup_world()
    # gridworld.save_scene()
    ##
    '''Run a random simulation / for debugging - not used in MCTS!!!'''
    trace = gridworld.trace
    for i in range(1,maxstep):
        gridworld.timestep = i
        print('Step {}'.format(i))
        # testers take step
        for pedestrian in gridworld.tester_peds:
            # currently pick a random enabled action
            action = choice([key for key in gridworld.enabled_actions_pedestrian(pedestrian).keys()])
            gridworld.pedestrian_take_step(pedestrian,action)
        for agent in gridworld.tester_cars:
            action = choice([key for key in gridworld.enabled_actions_car(agent).keys()])
            gridworld.tester_take_step(agent,action)
        # agents = self.sys_agents + self.tester_agents
        gridworld.print_intersection(gridworld.timestep)
        # ego takes step
        for ego_agent in gridworld.sys_agents:
            action = gridworld.strategy(ego_agent)
            gridworld.agent_take_step(ego_agent, action)
        # save the scene
        gridworld.print_intersection(gridworld.timestep)
        # check if we are done
        for agent in gridworld.sys_agents:
            if gridworld.is_terminal():
                print('Goal reached')
                # save the trace
                print(gridworld.trace)
                save_trace(filepath, gridworld.trace)
                return
    save_trace(filepath, gridworld.trace)


def append_trace(trace_dict, agent):
    '''Append the trace'''
    trace_dict["x"].append(agent.x)
    trace_dict["y"].append(agent.y)
    trace_dict["v"].append(agent.v)


def play_game(intersectionfile):
    '''Play the game using MCTS to find the strategy'''
    trace=[]
    tree = MCTS()
    # set the output path to save the trace
    output_dir = os.getcwd()+'/intersection/saved_traces/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'sim_trace.p'
    filepath = output_dir + filename
    # Set up the intersection gridworld
    gridworld = new_World(intersectionfile)
    trace = save_scene(gridworld,trace) # save the initial scene

    game_trace = [] # Same as ego_trace and env_trace condensed into one step with env going first
    k = 0 #  Time stamp
    # Initial step by the system:
    for sys_agent in gridworld.sys_agents:
        action = gridworld.strategy(sys_agent)
        gridworld.agent_take_step(sys_agent, 'n')

    # Initial step by testers:
    for pedestrian in gridworld.tester_peds:
        # currently pick a random enabled action
        # st()
        # action = choice([key for key in gridworld.enabled_actions_pedestrian(pedestrian).keys()])
        action = 'stay'
        gridworld.pedestrian_take_step(pedestrian,action)
    for test_agent in gridworld.tester_cars:
        # st()
        # action = choice([key for key in gridworld.enabled_actions_car(test_agent).keys()])
        action = 'stay'
        gridworld.tester_take_step(test_agent,action)

    gridworld.print_intersection(gridworld.timestep)

    while True:
        # Action for system under test
        for sys_agent in gridworld.sys_agents:
            action = gridworld.strategy(sys_agent)
            gridworld.agent_take_step(sys_agent, action)

        # for agent in gridworld.ego_agents:
        #     append_trace(ego_trace, agent)
        # game_trace.append(deepcopy(gridworld))
        grid_term = gridworld.is_terminal()
        trace = save_scene(gridworld,trace)
        gridworld.print_intersection(gridworld.timestep)
        # st()
        if grid_term:
            if k==0:
                print("Poor initial choices; no MCTS rollouts yet")
            else:
                print("Goal reached!")
            break
        else:
            k = k + 1
        gridworldnew = deepcopy(gridworld)
        for k in range(5):
            # print("Rollout: ", str(k+1))
            tree.do_rollout(gridworldnew)
        gridworldnew = tree.choose(gridworldnew) # Env action
        # st()
        # find which actions were chosen and take them
        actions = which_action(gridworld,gridworldnew) # stop here
        # st()
        for pedestrian in gridworld.tester_peds:
            gridworld.pedestrian_take_step(pedestrian,actions['ped'])
        for car in gridworld.tester_cars:
            gridworld.tester_take_step(car,actions['car'])
        # for agent in gridworld.env_agents:
        #     append_trace(env_trace, agent)
        trace = save_scene(gridworld,trace)
        gridworld.print_intersection(gridworld.timestep-1)
        grid_term = gridworld.is_terminal()
    save_trace(filepath,trace)
    return #ego_trace, env_trace, game_trace

def which_action(gridworld,gridworldnew):
    '''Find from chosen gridworld which action was chosen for each agent'''
    actions = []

    car_old = gridworld.tester_cars[0]
    ped_old = gridworld.tester_peds[0]

    car_new = gridworldnew.tester_cars[0]
    ped_new = gridworldnew.tester_peds[0]

    if car_old.y == car_new.y:
        car_act = 'stay'
    else:
        car_act = 's'

    if ped_old.cwloc == ped_new.cwloc:
        ped_act = 'stay'
    else:
        if ped_new.cwloc > ped_old.cwloc:
            ped_act = 'forward'
        elif ped_new.cwloc < ped_old.cwloc:
            ped_act = 'back'

    actions = dict()
    actions.update({'ped': ped_act})
    actions.update({'car': car_act})
    return actions

# Constructing trace:
def append_trace(trace_dict, agent):
    trace_dict["x"].append(agent.x)
    trace_dict["y"].append(agent.y)
    trace_dict["v"].append(agent.v)

if __name__ == '__main__':
    # run_random_sim(25)
    # st()
    intersectionfile = os.getcwd()+'/intersection/intersectionfile.txt'

    output_dir = os.getcwd()+'/highway_merge/saved_traces/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'sim_trace.p'
    filepath = output_dir + filename
    play_game(intersectionfile)
    # print("Ego trajectory")
    # print(ego_trace)
    # print("")
    # print("Environment trajectory")
    # print(env_trace)
