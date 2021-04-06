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
from copy import deepcopy

# global
Agent = namedtuple('Agent', 'name x y v goal')

class GridWorld:
  def __init__(self,lanes,width, initial_scene, ego_agents=None, env_agents=None, turn=None):
    self.lanes = lanes    # Number of one direction lanes stacked
    self.width = width    # number of gridcells in a lane
    self.initial_scene = initial_scene # dictionary of initial agent conditions
    if ego_agents is not None:
        self.ego_agents = ego_agents # empty dictionary of ego agents in system
    else:
        self.ego_agents = dict()
    
    if env_agents is not None:
        self.env_agents = env_agents # empty dictionary of env agents in system
    else:
        self.env_agents = dict()
    self.actions = {'move': (1,0), 'stay': (0,0), 'mergeR': (1,1), 'mergeL': (1,-1)} # possible actions
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
          agent = 'ag_env'
          enabled_actions = self.enabled_actions(self.env_agents[agent])
          children = set()
          count = 1
          for ai in enabled_actions:
              x,y = enabled_actions[ai]
              env_agents = {agent: Agent(name=self.env_agents[agent].name, x=x,y=y,v=self.env_agents[agent].v, goal=self.env_agents[agent].goal)}
              gi = GridWorld(self.lanes, self.width, self.initial_scene,ego_agents=self.ego_agents, env_agents=env_agents, turn="ego") 
              print("Env agent step possibility: ", str(count))
              gi.print_state()
              count = count+1
              children.add(gi)
      else:
          agent = 'ego'
          enabled_actions = self.enabled_actions(self.ego_agents[agent])
          children = set()
          count = 1
          for ai in enabled_actions:
              x,y = enabled_actions[ai]
              ego_agents = {agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)}
              gi = GridWorld(self.lanes, self.width, self.initial_scene,ego_agents=ego_agents, env_agents=self.env_agents) 
              print("Ego step possibility: ", str(count))
              gi.print_state()
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
    Agent = namedtuple('Agent', 'name x y v goal')
    for i,agent in enumerate(self.initial_scene):
      if agent.name[0:3] =='ego':
        self.ego_agents.update({agent.name : Agent(name=agent.name, x=agent.x,y=agent.y,v=agent.v, goal=agent.goal)})
      else:
        self.env_agents.update({agent.name : Agent(name=agent.name, x=agent.x,y=agent.y,v=agent.v, goal=agent.goal)})
    self.print_state()

  def ego_take_step(self):
    for agent in self.ego_agents:
      enabled_actions = self.enabled_actions(self.ego_agents[agent])
      if 'mergeR' in enabled_actions.keys(): # if you can merge - do it
        x,y = enabled_actions['mergeR']
        self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
      else:
        actionlist = []
        for action in enabled_actions.keys():
            actionlist.append(action)
        chosen_action = choice(actionlist)
        x,y = enabled_actions[chosen_action]
        self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
    self.print_state()
  
  def ego_take_input(self, action):
    for agent in self.ego_agents:
      enabled_actions = self.enabled_actions(self.ego_agents[agent])
      if action in enabled_actions.keys(): # if you can merge - do it
        x,y = enabled_actions[action]
        self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
      elif 'mergeR' in enabled_actions.keys():
        ran_act = choice(['mergeR', 'stay'])
        x,y = enabled_actions[ran_act]
        self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
      else:
        x,y = enabled_actions['stay']
        self.ego_agents.update({agent: Agent(name=self.ego_agents[agent].name, x=x,y=y,v=self.ego_agents[agent].v, goal=self.ego_agents[agent].goal)})
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
    agents = {**self.ego_agents, **self.env_agents}
    x,y = cellxy
    for agent in agents.keys():
      if agents[agent].x == x and agents[agent].y == y:
        return False
    return True

  def env_take_step(self, agent, action, print_st=1):
    enabled_actions = self.enabled_actions(agent)
    if action in enabled_actions:
      x,y = enabled_actions[action]
      self.env_agents.update({agent.name: Agent(name=agent.name, x=x,y=y,v=agent.v, goal=agent.goal)})
    else:
      x,y = enabled_actions['stay']
      self.env_agents.update({agent.name: Agent(name=agent.name, x=x,y=y,v=agent.v, goal=agent.goal)})
    #else: # the action is 'stay'
    if print_st:
        self.print_state()

  def reward(self):
    if not self.terminal:
      raise RuntimeError("reward called on nonterminal gridworld")
    else:
      agent = self.ego_agents["ego"]
      if agent.y == agent.goal:
          return agent.x
      elif agent.x == self.width:
          return 0 # No reward for causing the ego player to lose

  def is_terminal(self):
    agent = self.ego_agents["ego"]
    if agent.y == agent.goal or agent.x == self.width:
      self.terminal = True
    else:
      self.terminal = False
    return self.terminal

  def print_state(self):
    agents = {**self.ego_agents, **self.env_agents}
    for i in range(1,self.lanes+1):
      lanestr = []
      for k in range(1,self.width+1):
        #print('i,k = {0},{1}'.format(i,k))
        occupied = False
        for key in agents:
          if agents[key].x == k and agents[key].y == i:
            lanestr.append('|'+str(agents[key].name[0]))
            occupied = True
        if not occupied:
          lanestr.append('| ') # no car in this cell
      lanestr.append('| ')
      print(''.join(lanestr))
    print('\n')
    pass

def new_init_scene():
  Init_agent = namedtuple("Init_agent", ["name", "x", "y", "v", "goal"])
  ego_tuple = Init_agent(name ="ego", x = 1, y = 1, v=1, goal = 2)
  tester_tuple = Init_agent(name ="ag_env", x = 1, y = 2, v=1, goal = 2)
  return (ego_tuple, tester_tuple)

def new_World():
  init_scene = new_init_scene()
  return GridWorld(2, 10, init_scene)

def run_random_sim():
  # run a game
  gridworld = new_World()
  gridworld.setup_world()
  acts = ['mergeL','stay','move', 'mergeR']
  for i in range(0,2):
    print('Step {}'.format(i))
    gridworld.ego_take_step()
    for agent in gridworld.env_agents:
      gridworld.env_take_step(gridworld.env_agents[agent],np.random.choice(acts))
    #gridworld.print_state()
    for agent in gridworld.ego_agents:
      if gridworld.is_terminal():
        print('Goal reached')
        return

# Constructing trace:
def append_trace(trace_dict, agent):
    trace_dict["x"].append(agent.x)
    trace_dict["y"].append(agent.y)
    trace_dict["v"].append(agent.v)
# # Playing a game:
def play_game():
  tree = MCTS()
  #print(gridworld.print_state())
  gridworld = new_World()
  gridworld.setup_world()
  acts = ['mergeL','stay','move', 'mergeR']
  ego_trace = {"x": [], "y": [], "v": []}
  env_trace = {"x": [], "y": [], "v": []}
  append_trace(env_trace, gridworld.env_agents["ag_env"])
  append_trace(ego_trace, gridworld.ego_agents["ego"])
  
  game_trace = [] # Same as ego_trace and env_trace condensed into one step with env going first
  k = 0 #  Time stamp
  # Initial step by environment:
  for agent in gridworld.env_agents:
      gridworld.env_take_step(gridworld.env_agents[agent],np.random.choice(acts))
  append_trace(env_trace, gridworld.env_agents["ag_env"])
  while True:
    gridworld.ego_take_input('move')  # Ego action
    append_trace(ego_trace, gridworld.ego_agents["ego"])
    game_trace.append(deepcopy(gridworld))
    grid_term = gridworld.is_terminal()
    if grid_term:
      if k==0:
          print("Poor initial choices; no MCTS rollouts yet")
      if gridworld.width == gridworld.ego_agents["ego"].x and gridworld.ego_agents["ego"].y == 1:
          print('Did not merge; end of road')
      else:
          print("Goal reached; ego successfully merged!")
      break
    else:
      k = k+1
    for k in range(50):
      print("Rollout: ", str(k+1))
      tree.do_rollout(gridworld)
    gridworld = tree.choose(gridworld) # Env action
    append_trace(env_trace, gridworld.env_agents["ag_env"])
    grid_term = gridworld.is_terminal()
  
  return ego_trace, env_trace, game_trace


if __name__ == '__main__':
    ego_trace, env_trace, game_trace = play_game()
    print("Ego trajectory")
    print(ego_trace)
    print("")
    print("Environment trajectory")
    print(env_trace)
