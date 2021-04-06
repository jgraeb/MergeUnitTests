#############################################################################
#                                                                           #
# Basic class for grid world traffic simulation, separate from MCTS for now #
# Josefine Graebener, Apurva Badithela                                      #
# Caltech, March 2021                                                       #
#                                                                           #
#############################################################################
from collections import namedtuple
from random import choice
from mcts import MCTS, Node
import numpy as np
from scene import Scene
from agent import Agent
from map import Map
import _pickle as pickle
import os
from copy import deepcopy

# global
# Agent = namedtuple('AgentTuple', 'name x y v goal') # change everything to use Agent class instaed

class GridWorld:
    def __init__(self,lanes,width, initial_scene, ego_agents=None, env_agents=None, turn=None):
        self.lanes = lanes    # Number of one direction lanes stacked
        self.width = width    # number of gridcells in a lane
        self.initial_scene = initial_scene # dictionary of initial agent conditions
        if ego_agents is not None:
            self.ego_agents = ego_agents # empty dictionary of ego agents in system
        else:
            self.ego_agents = []

        if env_agents is not None:
            self.env_agents = env_agents # empty dictionary of env agents in system
        else:
            self.env_agents = []
        self.actions = {'move': (1,0), 'stay': (0,0), 'mergeR': (1,1), 'mergeL': (1,-1)} # possible actions
        self.trace = []
        self.map = Map(lanes,width)
        self.timestep = 0
        # Game conditions of the MCTS:
        if turn is not None:
            self.turn = turn
        else:
            self.turn = "env"
        self.terminal = False # This is when ego finally merges to cell 2; should be set to the ego agent tuple once merge is complete

   # Children are the successors of the env_agent actions
    def find_children(self):
        if self.terminal:
            return set()
        if self.turn=="env":
            #agent = 'ag_env'
            for agent in self.env_agents:
                enabled_actions = self.enabled_actions(agent)
                children = set()
                count = 1
                for ai in enabled_actions:
                    x,y = enabled_actions[ai]
                    env_agents = [Agent(name=agent.name, x=x,y=y,v=agent.v, goal=agent.goal)]
                    gi = GridWorld(self.lanes, self.width, self.initial_scene,ego_agents=self.ego_agents, env_agents=env_agents, turn="ego")
                    #print("Env agent step possibility: ", str(count))
                    #gi.print_state()
                    count = count+1
                    children.add(gi)
        else:
            for agent in self.ego_agents:
                #agent = 'ego'
                enabled_actions = self.enabled_actions(agent)
                children = set()
                count = 1
                for ai in enabled_actions:
                    x,y = enabled_actions[ai]
                    ego_agents = [Agent(name=agent.name, x=x,y=y,v=agent.v, goal=agent.goal)]
                    gi = GridWorld(self.lanes, self.width, self.initial_scene,ego_agents=ego_agents, env_agents=self.env_agents)
                    #print("Ego step possibility: ", str(count))
                    #gi.print_state()
                    count = count+1
                    children.add(gi)
        return children

#  is a gridworld object
    def find_random_child(self):
        if self.terminal:
            return None
        children = self.find_children()
        ran_child = choice(list(children))
        return ran_child

    def setup_world(self):
        #Agent = namedtuple('Agent', 'name x y v goal')
        for i,agent in enumerate(self.initial_scene):
            if agent.name[0:3] =='ego':
                self.ego_agents.append(agent)
            else:
                self.env_agents.append(agent)
        self.print_state()

    def ego_take_input(self, action):
        for agent in self.ego_agents:
            enabled_actions = self.enabled_actions(agent)
            if action in enabled_actions.keys():
                x,y = enabled_actions[action]
                agent.x = x
                agent.y = y
                #self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
            elif 'move' in enabled_actions.keys():
                ran_act = choice(['move', 'stay'])
                x,y = enabled_actions[ran_act]
                agent.x = x
                agent.y = y
                # self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
            else:
                x,y = enabled_actions['stay']
                agent.x = x
                agent.y = y
                # self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
        #self.print_state()


    def env_take_step(self, agent, action, print_st=0):
        for agent in self.ego_agents:
            enabled_actions = self.enabled_actions(agent)
            if 'mergeR' in enabled_actions.keys(): # if you can merge - do it
                x,y = enabled_actions['mergeR']
                # agent.setxy(x,y)
                agent.x = x
                agent.y = y
                # agent.step_mergeR()
                #self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
            else:
                actionlist = []
                for action in enabled_actions.keys():
                    actionlist.append(action)
                    chosen_action = choice(actionlist)
                    x,y = enabled_actions[chosen_action]
                    agent.x = x
                    agent.y = y
                    #self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
            print('ego took step to {0},{1}'.format(agent.x,agent.y))
        if print_st:
            self.print_state()

    def reward(self):
        if not self.terminal:
            raise RuntimeError("reward called on nonterminal gridworld")
        else:
            for agent in self.ego_agents:
            #agent = self.ego_agents["ego"]
                if agent.y == agent.goal:
                    return agent.x
                elif agent.x == self.width:
                    return 0 # No reward for causing the ego player to lose

    def enabled_actions(self,agent):
        enabled_actions = dict()
        x = agent.x
        y = agent.y
        for action in self.actions.keys():
            move_x,move_y = self.actions[action]
            act_x = x+move_x
            act_y = y+move_y
            if self.is_cell_free((act_x,act_y)) and act_y in range(1,3):
                enabled_actions.update({action: (act_x,act_y)})
            enabled_actions.update({'stay': (x,y)})
        return enabled_actions

    def is_cell_free(self, cellxy):
        agents = self.ego_agents + self.env_agents
        x,y = cellxy
        for agent in agents:
            if agent.x == x and agent.y == y:
                return False
        return True

    def env_take_step(self, agent, action):
        enabled_actions = self.enabled_actions(agent)
        if action in enabled_actions:
            x,y = enabled_actions[action]
            agent.x = x
            agent.y = y
        else:
            x,y = enabled_actions['stay']
            agent.x = x
            agent.y = y


    def is_terminal(self):
        for agent in self.ego_agents:
            if agent.y == agent.goal or agent.x == self.width:
                self.terminal = True
            else:
                self.terminal = False
        return self.terminal

    def check_terminal(self, agent):
        if agent.y == agent.goal:
            self.terminal = True
        else:
            self.terminal = False
        return self.terminal

    def print_state(self):
        agents = self.ego_agents + self.env_agents
        for i in range(1,self.lanes+1):
            lanestr = []
            for k in range(1,self.width+1):
                occupied = False
                for agent in agents:
                    if agent.x == k and agent.y == i:
                        lanestr.append('|'+str(agent.name[0]))
                        occupied = True
                if not occupied:
                    lanestr.append('| ') # no car in this cell
            lanestr.append('| ')
            print(''.join(lanestr))
        print('\n')

def save_trace(filename,trace):
    print('Saving trace in pkl file')
    #import pdb; pdb.set_trace()
    with open(filename, 'wb') as pckl_file:
        pickle.dump(trace, pckl_file)

def save_scene(gridworld,trace):
    #import pdb; pdb.set_trace()
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
    return trace

def new_init_scene():
    ego_tuple = Agent(name ="ego", x = 1, y = 1, v=1, goal = 2)
    tester_tuple = Agent(name ="ag_env", x = 1, y = 2, v=1, goal = 2)
    return (ego_tuple, tester_tuple)

def new_World():
    init_scene = new_init_scene()
    return GridWorld(2, 10, init_scene)

def run_random_sim(maxstep):
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
     trace_dict["x"].append(agent.x)
     trace_dict["y"].append(agent.y)
     trace_dict["v"].append(agent.v)

def play_game():
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
    #print(gridworld.print_state())
    # gridworld = new_World()
    # gridworld.setup_world()
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
        trace = save_scene(gridworld,trace) # save first env action
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
        for k in range(50):
            #print("Rollout: ", str(k+1))
            tree.do_rollout(gridworldnew)
        gridworldnew = tree.choose(gridworldnew) # Env action
        #import pdb; pdb.set_trace()
        newx = gridworldnew.env_agents[0].x
        oldx = gridworld.env_agents[0].x
        newy = gridworldnew.env_agents[0].y
        oldy = gridworld.env_agents[0].y
        if newx == oldx:
            action = 'stay'
        elif newy != oldy:
            action = 'mergeR'
        else:
            action = 'move'
        for agent in gridworld.env_agents:
            gridworld.env_take_step(agent,action)
        for agent in gridworld.env_agents:
            append_trace(env_trace, agent)
        trace = save_scene(gridworld,trace)
        gridworld.print_state()
        grid_term = gridworld.is_terminal()
    save_trace(filepath,trace)
    return ego_trace, env_trace, game_trace

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
