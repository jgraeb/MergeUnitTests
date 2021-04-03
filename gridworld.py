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

# global
#Agent = namedtuple('Agent', 'name x y v goal') # change everything to use Agent class instaed

class GridWorld:
    def __init__(self,lanes,width, initial_scene):
        self.lanes = lanes    # Number of one direction lanes stacked
        self.width = width    # number of gridcells in a lane
        self.initial_scene = initial_scene # dictionary of initial agent conditions
        self.ego_agents = [] # empty dictionary of ego agents in system
        self.env_agents = [] # empty dictionary of env agents in system
        self.actions = {'move': (1,0), 'stay': (0,0), 'mergeR': (1,1), 'mergeL': (1,-1)} # possible actions
        self.trace = []
        self.map = Map(lanes,width)
        self.timestep = 0
        self.trace = []
        # Game conditions of the MCTS:
        self.turn = "env"
        self.terminal = False # This is when ego finally merges to cell 2; should be set to the ego agent tuple once merge is complete

    def setup_world(self):
        #Agent = namedtuple('Agent', 'name x y v goal')
        for i,agent in enumerate(self.initial_scene):
            if agent.name[0:3] =='ego':
                self.ego_agents.append(agent)
            else:
                self.env_agents.append(agent)
        self.print_state()

    def ego_take_step(self):
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
        self.print_state()

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
            #self.env_agents.update({agent.name: Agent(name=agent.name, x=x,y=y,v=agent.v, goal=agent.goal)})
        else:
            x,y = enabled_actions['stay']
            agent.x = x
            agent.y = y
            #self.env_agents.update({agent.name: Agent(name=agent.name, x=x,y=y,v=agent.v, goal=agent.goal)})
            #else: # the action is 'stay'
        print('env took step to {0},{1}'.format(agent.x,agent.y))
        self.print_state()

  # def reward(self):
  #   if not self.terminal:
  #     raise RuntimeError("reward called on nonterminal gridworld")
  #   else:
  #     return self.ego_agents["ego"].x

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
                #print('i,k = {0},{1}'.format(i,k))
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

    def save_trace(self,filename):
        with open(filename, 'wb+') as pckl_file:
            pickle.dump(self.trace, pckl_file)

    def save_scene(self):
        print('Saving scene {}'.format(self.timestep))
        ego_snapshot = []
        env_snapshot = []
        for agent in self.ego_agents:
            ego_snapshot.append((agent.name,agent.x,agent.y))
        for agent in self.env_agents:
            env_snapshot.append((agent.name,agent.x,agent.y))
        current_scene = Scene(self.timestep, self.map, ego_snapshot, env_snapshot)
        self.trace.append(current_scene)
        self.timestep += 1

  # # Passing current state of gridworld to Rose simulator:
  # # ToDo: Update this function
  # def to_sim_frame(self):
  #   # to_char = lambda v: ("X" if v is True else ("O" if v is False else " "))
  #   # rows = [
  #   #     [to_char(board.tup[3 * row + col]) for col in range(3)] for row in range(3)
  #   # ]
  #   # return (
  #   #     "\n  1 2 3\n"
  #   #     + "\n".join(str(i + 1) + " " + " ".join(row) for i, row in enumerate(rows))
  #   #     + "\n"
  #   # )
  #   pass

def new_init_scene():
    ego_tuple = Agent(name ="ego", x = 1, y = 1, v=1, goal = 2)
    tester_tuple = Agent(name ="ag_env", x = 1, y = 2, v=1, goal = 2)

    # Init_agent = namedtuple("Init_agent", ["name", "x", "y", "v", "goal"])
    # ego_tuple = Init_agent(name ="ego", x = 1, y = 1, v=1, goal = 2)
    # tester_tuple = Init_agent(name ="ag_env", x = 1, y = 2, v=1, goal = 2)
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


# # Playing a game:
# def play_game(gridworld):
#   tree = MCTS()
#   print(gridworld.print_state())
#   while True:
#     gridworld.ego_take_step()
#     gridworld.reached_goal(gridworld.ego_agents["ego"])
#     if gridworld.terminal:
#         break
#
#     for _ in range(50):
#         tree.do_rollout(gridworld)
#     gridworld = tree.choose(gridworld)

if __name__ == '__main__':
    run_random_sim(10)
