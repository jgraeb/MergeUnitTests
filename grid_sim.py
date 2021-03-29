from collections import namedtuple
from random import choice
from mcts import MCTS, Node
import numpy as np

class GridWorld:
  def __init__(self,lanes,width, initial_scene):
    self.lanes = lanes    # Number of one direction lanes stacked
    self.width = width    # number of gridcells in a lane
    self.initial_scene = initial_scene # dictionary of initial agent conditions
    self.ego_agents = dict() # empty dictionary of ego agents in system
    self.env_agents = dict() # empty dictionary of env agents in system
    self.actions = [{'move': (1,0)}, {'stay': (0,0)}, {'merge': (1,1)}] # possible actions
    # Game conditions of the MCTS:
    self.turn = "env"
    self.terminal = False # This is when ego finally merges to cell 2; should be set to the ego agent tuple once merge is complete
    
  def setup_world(self):
    Agent = namedtuple('Agent', 'name x y v goal')
    for i,agent in enumerate(self.initial_scene):
      if agent.name[0:3] =='ego':
        self.ego_agents.update({agent.name : Agent(name=agent.name, x=agent.x,y=agent.y,v=agent.v, goal=agent.goal)})
      else:
        self.env_agents.update({agent.name : Agent(name=agent.name, x=agent.x,y=agent.y,v=agent.v, goal=agent.goal)})
    self.print_state()

  def ego_take_step(self):
    for agent in self.ego_agents.keys():
      enabled_actions = self.enabled_actions(agent)
      if enabled_actions.has_key('merge'): # if you can merge - do it
        x,y = enabled_actions['merge']
        agent.x = x
        agent.y = y
      else:
        chosen_action = choice(enabled_actions.keys())
        x,y = enabled_actions[chosen_action]
        agent.x = x
        agent.y = y
    self.print_state()

  def enabled_actions(self,agent):
    enabled_actions = dict()
    x = agent.x
    y = agent.y
    for action in self.actions.keys():
      move_x,move_y = self.actions[action]
      act_x = x+move_x
      act_y = y+move_y
      if self.is_cell_free(act_x,act_y):
        enabled_actions.update({action: (act_x,act_y)})
    enabled_actions.update({'stay': (x,y)})
    return enabled_actions

  def is_cell_free(self, cellxy):
    agents = self.ego_agents | self.env_agents 
    x,y = cellxy
    for agent in agents.keys():
      if agent.x == x and agent.y == y:
        return False
    return True

  def env_take_step(self, agent, action):
    enabled_actions = self.enabled_actions(agent)
    x,y = enabled_actions[action]
    agent.x = x
    agent.y = y
    self.print_state()

  def reward(self):
    if not self.terminal:
      raise RuntimeError("reward called on nonterminal gridworld")
    else:
      return self.ego_agents["ego"].x

  def reached_goal(self, agent):
    if agent.y == agent.goal:
      self.terminal = True
    else:
      self.terminal = False

  def print_state(self):
    agents = {**self.ego_agents, **self.env_agents}
    for i in range(0,self.lanes):
      for k in range(0,self.width):
        for key in agents:
          if agents[key].x == k and agents[key].y == i:
            print('|'+str(agents[key].name[0]),end='') # plot first letter of car name (i.e. 'e' or 'a')
          else:
            print('| ',end='') # no car in this cell
      print('|')
      print('\n')
    pass
  
  # Passing current state of gridworld to Rose simulator:
  # ToDo: Update this function
  def to_sim_frame(self):
    # to_char = lambda v: ("X" if v is True else ("O" if v is False else " "))
    # rows = [
    #     [to_char(board.tup[3 * row + col]) for col in range(3)] for row in range(3)
    # ]
    # return (
    #     "\n  1 2 3\n"
    #     + "\n".join(str(i + 1) + " " + " ".join(row) for i, row in enumerate(rows))
    #     + "\n"
    # )
    pass

def new_init_scene():
  Init_agent = namedtuple("Init_agent", ["name", "x", "y", "v", "goal"])
  ego_tuple = Init_agent(name ="ego", x = 1, y = 1, v=1, goal = 2)
  tester_tuple = Init_agent(name ="ag_env", x = 1, y = 2, v=1, goal = 2)
  return (ego_tuple, tester_tuple)

def new_World():
  init_scene = new_init_scene()
  return GridWorld(2, 10, init_scene)

def debug_sim():
  # play a game
  gridworld = new_World()
  gridworld.setup_world()
  for i in range(0,10):
    print('Step {}'.format(i))
    gridworld.ego_take_step()
    for agent in gridworld.env_agents:
      gridworld.env_take_step(agent,np.random.choice(1,2,3))
    gridworld.print_state()

# Playing a game:
def play_game(gridworld):
  tree = MCTS()
  print(gridworld.print_state())
  while True:
    gridworld.ego_take_step()
    gridworld.reached_goal(gridworld.ego_agents["ego"])
    if gridworld.terminal:
        break
    # You can train as you go, or only at the beginning.
    # Here, we train as we go, doing fifty rollouts each turn.
    for _ in range(50):
        tree.do_rollout(gridworld)
    gridworld = tree.choose(gridworld)
  