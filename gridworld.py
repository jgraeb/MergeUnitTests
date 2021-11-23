#############################################################################
#                                                                           #
# Basic class for grid world traffic simulation                             #
# Josefine Graebener, Apurva Badithela                                      #
# Caltech, March 2021                                                       #
#                                                                           #
#############################################################################
from random import choice
import numpy as np
from scene import Scene
from agent import Agent
from map import Map
from winning_set.winning_set import WinningSet, specs_for_entire_track, make_grspec, create_shield
import _pickle as pickle
import os
from copy import deepcopy
from ipdb import set_trace as st
from winning_set.merge_receding_horizon_winsets import get_tester_states_in_winsets, check_system_states_in_winset
from helper import save_ws_comp_result, load_ws
from test_parameters import TRACKLENGTH, MERGE_SETTING

def synthesize_guide():
    MERGE_SETTING = 'between'
    # read pickle file - if not there save a new one
    try:
        print('Checking for the saved guide')
        Wij, Vij_dict, state_tracker, ver2st_dict = load_ws()
        print('Guide loaded successfully')

    except:
        print('Synthesizing the guide')
        Wij, Vij_dict, state_tracker, ver2st_dict = get_tester_states_in_winsets(TRACKLENGTH, MERGE_SETTING)
        save_ws_comp_result(Wij, Vij_dict, state_tracker, ver2st_dict)
    return Wij, Vij_dict, state_tracker, ver2st_dict

Wij, Vij_dict, state_tracker, ver2st_dict = synthesize_guide()

class GridWorld:
    def __init__(self,lanes,width, initial_scene, ego_agents=None, env_agents=None, turn=None):
        self.lanes = lanes    # Number of one direction lanes stacked
        self.width = width    # number of gridcells in a lane
        self.initial_scene = initial_scene # dictionary of initial agent conditions
        if ego_agents is not None:
            self.ego_agents = ego_agents #
        else:
            self.ego_agents = [] # empty list of ego agents in system

        if env_agents is not None:
            self.env_agents = env_agents
        else:
            self.env_agents = [] # empty list of env agents in system
        self.actions = {'move': (1,0), 'stay': (0,0), 'mergeR': (1,1), 'mergeL': (1,-1)} # possible actions for ego
        self.env_actions = {'move': (1,0), 'stay': (0,0)} # possible actions for each tester
        self.trace = []
        self.map = Map(lanes,width)
        self.timestep = 0
        # Game conditions of the MCTS:
        if turn is not None:
            self.turn = turn
        else:
            self.turn = "env"
        self.terminal = False # This is when ego finally merges to cell 2; should be set to the ego agent tuple once merge is complete
        self.shield_dict = None
        # self.w_set = None
        # self.aut, self.winning_set = self.synthesize_shield()
        # self.Wij, self.Vij_dict, self.state_tracker, self.ver2st_dict = self.synthesize_guide()


    '''-----Basic gridworld functions-----'''

    def setup_world(self):
        '''Initializing the gridworld'''
        for i,agent in enumerate(self.initial_scene):
            if agent.name[0:3] =='sys':
                self.ego_agents.append(agent)
            else:
                self.env_agents.append(agent)
        self.print_state()

    def take_next_tester_step(self,action_comb,agent_list):
        agent_list_copy = deepcopy(agent_list)
        for i,action in enumerate(action_comb):
            agent_list_copy[i][1] = agent_list_copy[i][1]+self.env_actions[action][0]
            agent_list_copy[i][2] = agent_list_copy[i][2]+self.env_actions[action][1]
        return agent_list_copy

    def take_next_step(self,agent,action,agent_list, check=False, debug = False):
        '''Given a list of agent positions, update the chosen agent position after the step'''
        # find agent in agentlist
        i = agent_list.index(agent)
        # make this move
        agentpos = (agent[1],agent[2])
        enabled_actions = self.enabled_actions_from_loc(agentpos, agent_list)
        agent_list[i][1] = enabled_actions[action][0]
        agent_list[i][2] = enabled_actions[action][1]
        # return agent_list
        # check if allowed by shield here
        # state = {'x':,'y':,'x1':,'y1':, 'x2':, 'y2':}
        if check:
            if debug:
                st()
            # print("trying to check")
            # st()
            state = {'x': self.ego_agents[0].x, 'y': self.ego_agents[0].y, 'x1': agent_list[0][1], 'y1': agent_list[0][2], 'x2': agent_list[1][1], 'y2': agent_list[1][2]}
            origin_state = {'x':self.ego_agents[0].x,'y':self.ego_agents[0].y,'x1':self.env_agents[0].x,'y1':self.env_agents[0].y, 'x2':self.env_agents[1].x, 'y2':self.env_agents[1].y}
            # make sure all agents stay on the track:
            for tester in self.env_agents:
                if tester.x > TRACKLENGTH:
                    return None
            # check for the guide filter
            if self.check_guide(origin_state, state, debug):
                return agent_list
            else:
                return None
        else:
            return agent_list

    def get_actions(self,agent,agent_list):
        '''get all possible actions for an agent from its position'''
        agentpos = (agent[1],agent[2])
        enabled_actions = self.enabled_actions_from_loc(agentpos, agent_list)
        return enabled_actions

    def check_if_spec_is_fulfilled(self,agentlist):
        # st()
        state = {'x':self.ego_agents[0].x ,'y':self.ego_agents[0].y ,'x1':agentlist[0][1],'y1':agentlist[0][2], 'x2':agentlist[1][1], 'y2':agentlist[1][2]}
        x = state['x']
        y = state['y']
        x1 = state['x1']
        y1 = state['y1']
        x2 = state['x2']
        y2 = state['y2']
        # st()

        if (((x+1,y+1) == (x1,y1) or (x+1,y+1) == (x2,y2)) or (x==x2 and x+2==x1)) and x1<=x2+2:# and  x1<=x2+2:
            # st()
            # if (x==x2 and x+2==x1):
                # print('A gap opened')
            return True
        else:
            # st()
            return False


    def get_relative_position(self):
        ego_x = self.ego_agents[0].x
        ego_y = self.ego_agents[0].y
        i = 0
        num_agents = len(self.env_agents)
        agent_x = []
        agent_y = []
        for agent in self.env_agents:
            agent_x.append(agent.x - ego_x)
            agent_y.append(agent.y - ego_y)
        # statedict = {'x': 0, 'y': 0, 'x1': agent_x[0], 'y1': agent_y[0], 'x2': agent_x[1], 'y2': agent_y[1]}
        statetup = (agent_x[0],agent_y[0],agent_x[1],agent_y[1])
        return statetup

    def synthesize_shield(self):
        accepted_states = create_shield()
        # st()
        # tester_actions = {'move': (1,0), 'stay': (0,0)}
        # find which actions are ok from which state
        shield_dict = dict()
        for state in accepted_states:
            x = state['x']
            y = state['y']
            x1 = state['x1']
            y1 = state['y1']
            x2 = state['x2']
            y2 = state['y2']
            # enabled_actions = dict()
            allowed_actions = []
            for action1 in self.env_actions.keys():
                new_t1 = tuple([sum(bb) for bb in zip(self.env_actions[action1], (x1,y1))])
                for action2 in self.env_actions.keys():
                    new_t2 = tuple([sum(bb) for bb in zip(self.env_actions[action2], (x2,y2))])
                    if new_t1 != new_t2 and new_t1!= (x,y) and new_t2!= (x,y):
                        newstate = {'x':0,'y':0,'x1':new_t1[0],'y1':new_t1[1],'x2':new_t2[0],'y2':new_t2[1]}
                        #
                        if newstate in accepted_states:
                            allowed_actions.append([action1,action2])
            state_tup = (x1,y1,x2,y2)
            shield_dict.update({state_tup: allowed_actions})
        # st()
        return shield_dict

    def check_next_states_in_shield(self):
        accepted_states = create_shield()
        rel_pos_tup = self.get_relative_position()
        allowed_actions = []
        for action1 in self.env_actions.keys():
            new_t1 = tuple([sum(bb) for bb in zip(self.env_actions[action1], (rel_pos_tup[0],rel_pos_tup[1]))])
            for action2 in self.env_actions.keys():
                new_t2 = tuple([sum(bb) for bb in zip(self.env_actions[action2], (rel_pos_tup[2],rel_pos_tup[3]))])
                if new_t1 != new_t2 and new_t1!= (0,0) and new_t2!= (0,0):
                    newstate = {'x':0,'y':0,'x1':new_t1[0],'y1':new_t1[1],'x2':new_t2[0],'y2':new_t2[1]}
                    if newstate in accepted_states:
                        allowed_actions.append([action1,action2])
        self.shield_dict.update({rel_pos_tup:allowed_actions})


    def check_if_state_in_winning_set(self,agent_list):
        # st()
        w_set = WinningSet()
        state = self.map_to_state(agent_list)
        #state = {'x': 1, 'y': 1, 'x1': 3, 'y1':2, 'x2':1, 'y2': 2}  # To check if a state is in the winning set, pass all values in dictionary form. Each dictionary corresponds to one state.
        check_bdd = w_set.check_state_in_winset(self.aut, self.winning_set, state) # Check-bdd is a boolean. True implies that state is in the winning set.
        st()
        if check_bdd:
            # print("State is in the winning set")
            return True
        else:
            # print("State is not in the winning set")
            return False

    def load_shield(self): # pass spec to winning set analysis here
        if not self.shield_dict:
            w_set = WinningSet()
            self.shield_dict = self.synthesize_shield()
            self.shield_dict.update({ (0, 1, -2, 1):[['move', 'move']]})


    def check_guide(self, origin_state, state, debug = False):
        if debug:
            st()
        in_ws = check_system_states_in_winset(origin_state, state, ver2st_dict, state_tracker, Wij, debug)
        return in_ws


    def map_to_state(self, agentlist):
        # automate this
        statedict = {'x': self.ego_agents[0].x, 'y': self.ego_agents[0].y, 'x1': agentlist[0][1], 'y1': agentlist[0][2], 'x2': agentlist[1][1], 'y2': agentlist[1][2]}
        return statedict

    def shield(self, enabled_actions):
        #
        if not self.shield_dict:
            self.load_shield()
        # find relative positions of agents
        rel_pos_tup = self.get_relative_position()
        # run enabled actions through dict
        # find all children states of this state
        # check if they are allowed in the shield
        st()

        ok_actions = self.shield_dict[rel_pos_tup]
        shielded_actions = dict()
        for action in ok_actions:
            if action in enabled_actions:
                shielded_actions.update({action: enabled_actions[action]})
        return shielded_actions

    def ego_take_input(self, action):
        '''Ego agent takes the step'''
        for agent in self.ego_agents:
            enabled_actions = self.enabled_actions(agent)
            if action in enabled_actions.keys():
                x,y = enabled_actions[action]
                agent.x = x
                agent.y = y
            elif 'move' in enabled_actions.keys():
                ran_act = choice(['move', 'stay'])
                x,y = enabled_actions[ran_act]
                agent.x = x
                agent.y = y
            else:
                x,y = enabled_actions['stay']
                agent.x = x
                agent.y = y

    def enabled_actions_for_2_testers(self):
        # st()
        # find relative state of current GridWorld
        rel_tup = self.get_relative_position()
        # find which actions are ok from shield
        self.load_shield()
        try:
            ok_actions = self.shield_dict[rel_tup]
        except:
            # if state is not in shield dict, check if there is an action leading to shield dict
            self.check_next_states_in_shield()
            ok_actions = self.shield_dict[rel_tup]
        return ok_actions


    def enabled_actions_from_loc(self,agentpos,agent_list):
        '''Find the possible actions for an agent from its position'''
        x = agentpos[0]
        y = agentpos[1]
        enabled_actions = dict()
        for action in self.env_actions.keys():
            move_x,move_y = self.env_actions[action]
            act_x = x + move_x
            act_y = y + move_y
            if self.is_cell_free((act_x,act_y),agent_list) and act_y in range(1,3):
                enabled_actions.update({action: (act_x,act_y)})
            enabled_actions.update({'stay': (x,y)})
        # st()
        # shielded_actions = self.shield(enabled_actions)
        # st()
        return enabled_actions

    def enabled_actions(self,agent):
        '''Find possible actions for agent'''
        enabled_actions = dict()
        x = agent.x
        y = agent.y
        for action in self.actions.keys():
            move_x,move_y = self.actions[action]
            act_x = x+move_x
            act_y = y+move_y
            if self.is_cell_free((act_x,act_y)) and act_y in range(1,3):
                enabled_actions.update({action: (act_x,act_y)})
            enabled_actions.update({'stay': (x,y)}) # stay is always included
        return enabled_actions

    def enabled_system_actions(self,agent):
        '''Find possible actions for system agent'''
        enabled_actions = dict()
        x = agent.x
        y = agent.y

        # check for merge
        move_x, move_y = self.actions['mergeR']
        act_x = x + move_x
        act_y = y + move_y
        if self.is_cell_free((act_x,act_y)) and act_y in range(1,3):
            enabled_actions.update({'mergeR': (act_x,act_y)})
            return enabled_actions
        for action in self.actions.keys():
            move_x,move_y = self.actions[action]
            act_x = x+move_x
            act_y = y+move_y
            if self.is_cell_free((act_x,act_y)) and act_y in range(1,3):
                enabled_actions.update({action: (act_x,act_y)})
            enabled_actions.update({'stay': (x,y)}) # stay is always included
        return enabled_actions


    def is_cell_free(self, cellxy, agent_list = None):
        '''check if the cell is free'''
        x,y = cellxy
        if not agent_list:
            agents = self.ego_agents + self.env_agents
            for agent in agents:
                if agent.x == x and agent.y == y:
                    return False
        else:
            for pos in agent_list: # check all env agents
                if pos[1] == x and pos[2] == y:
                    return False
            for agent in self.ego_agents:
                if agent.x == x and agent.y == y:
                    return False
        return True

    def agent_take_step(self,agent,action):
        '''For known action, take the step (used for find children in tree search)'''
        enabled_actions = self.enabled_actions(agent)
        x,y = enabled_actions[action]
        agent.x = x
        agent.y = y

    def env_take_step(self, agent, action):
        '''Take the step for env, used for actually taking the actions during execution'''
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
        '''Returns if the state is terminal'''
        for agent in self.ego_agents:
            if agent.y == agent.goal or agent.x == self.width:
                self.terminal = True
            else:
                self.terminal = False
        return self.terminal

    def check_terminal(self, agent):
        '''Returns if the state is terminal'''
        if agent.y == agent.goal:
            self.terminal = True
        else:
            self.terminal = False
        return self.terminal

    def print_state(self):
        '''Print the current state of all agents in the terminal'''
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

    ''' MCTS functions '''

    def find_all_next_steps(self, agent_list_original):
        # find all possible action combinations for node
        actions = self.enabled_actions_for_2_testers()
        list_of_agentlists_mod = []
        for action_comb in actions:
            agent_list_copy = self.take_next_tester_step(action_comb,agent_list_original)
            list_of_agentlists_mod.append(agent_list_copy)
        return list_of_agentlists_mod

    def get_children_gridworlds_v2(self):
        # st()
        '''Find all children nodes from the current node for env action next'''
        # prep the agent data
        ego_pos = (self.ego_agents[0].x, self.ego_agents[0].y)
        agent_list_original = [[agent.name, agent.x, agent.y, agent.v, agent.goal] for agent in self.env_agents]
        agent_list_original = sorted(agent_list_original, key = lambda item: item[1]) # sorted by x location
        agent_list_original.reverse()
        list_of_agentlists = self.find_all_next_steps(agent_list_original)
        list_of_gridworlds = [make_gridworld(agentlist,self.ego_agents) for agentlist in list_of_agentlists]
        return list_of_gridworlds
        pass

    def get_children_gridworlds(self,debug=False):
        if debug:
            st()
        # st()
        '''Find all children nodes from the current node for env action next'''
        # prep the agent data
        ego_pos = (self.ego_agents[0].x, self.ego_agents[0].y)
        agent_list_original = [[agent.name, agent.x, agent.y, agent.v, agent.goal] for agent in self.env_agents]
        agent_list_original = sorted(agent_list_original, key = lambda item: item[1]) # sorted by x location
        agent_list_original.reverse()
        # list_of_agentlists = [agent_list_original]
        # do all combinations of actions that are possible and then check
        list_of_agentlists_mod = []
        # pick action for first agent
        agent1 = agent_list_original[0]
        agent2 = agent_list_original[1]
        actions1 = self.get_actions(agent1,agent_list_original)
        for action1 in actions1:
            agent_list_copy = deepcopy(agent_list_original)
            agent_list_copy = self.take_next_step(agent1,action1,agent_list_copy)
            actions2 = self.get_actions(agent2,agent_list_copy)
            for action2 in actions2:
                agent_list_copy2 = deepcopy(agent_list_copy)
                check = True
                agent_list_copy2 = self.take_next_step(agent2,action2,agent_list_copy2,check, debug)
                if agent_list_copy2 is not None:
                    # print('A gridworld child was found')
                    list_of_agentlists_mod.append(agent_list_copy2)
        list_of_agentlists = deepcopy(list_of_agentlists_mod)
        list_of_gridworlds = [make_gridworld(agentlist,self.ego_agents) for agentlist in list_of_agentlists]
        # st()
        return list_of_gridworlds

    def get_children_gridworlds_backup(self,debug=False):
        if debug:
            st()
        # st()
        '''Find all children nodes from the current node for env action next'''
        # prep the agent data
        ego_pos = (self.ego_agents[0].x, self.ego_agents[0].y)
        agent_list_original = [[agent.name, agent.x, agent.y, agent.v, agent.goal] for agent in self.env_agents]
        agent_list_original = sorted(agent_list_original, key = lambda item: item[1]) # sorted by x location
        agent_list_original.reverse()
        list_of_agentlists = [agent_list_original]
        for i in range(0,len(agent_list_original)):
            list_of_agentlists_mod = []
            for agent_list in list_of_agentlists:
                # st()
                agent = agent_list[i]
                actions = self.get_actions(agent,agent_list)
                for action in actions:
                    agent_list_copy = deepcopy(agent_list)
                    agent_list_copy = self.take_next_step(agent,action,agent_list_copy)
                    if agent_list_copy is not None:
                        print('A gridworld child was found')
                        list_of_agentlists_mod.append(agent_list_copy)
            list_of_agentlists = list_of_agentlists_mod
        list_of_gridworlds = [make_gridworld(agentlist,self.ego_agents) for agentlist in list_of_agentlists]
        return list_of_gridworlds

    def find_random_child(self):
        # print('Finding a random child of node'+str(self.ego_agents[0].x)+str(self.ego_agents[0].y)+str(self.env_agents[0].x)+str(self.env_agents[1].x))
        '''Pick a random child node'''
        if self.terminal:
            return None

        children = self.find_children()
        # print_child_gridworlds(children)
        # st()

        try:
            ran_child = choice(list(children))
            # print('picked a child')
        except:
            st()
            debug = True
            children = self.find_children(debug)

        # print('Picked:')
        # ran_child.print_state()
        return ran_child

    def spec_check(self):
        '''Check if test spec is satisfied'''
        tester_list = []
        ego_list = []
        for tester in self.env_agents:
            tester_list.append((tester.x,tester.y))
        for ego in self.ego_agents:
            ego_list.append((ego.x,ego.y))
        for ego in ego_list:
            if (ego[0]-1,ego[1]) in tester_list and (ego[0]+1,ego[1]) in tester_list:
                return True
        return False

    def reward(self):
        '''Get reward for run'''
        # comp_reward = 5
        if not self.terminal:
            raise RuntimeError("reward called on nonterminal gridworld")
        else:
            # if self.spec_check():
            for agent in self.ego_agents:
            #agent = self.ego_agents["ego"]
                if agent.y == agent.goal:
                    # print('returned reward '+str(agent.x)+' for node '+str(self.ego_agents[0].x)+str(self.ego_agents[0].y))
                    return agent.x*10
                elif agent.x == self.width:
                    return 0 # No reward for causing the ego player to lose


    def find_children(self, debug = False):
        # st()
        '''Find all children for a node'''
        if self.terminal:
            return set()
        if self.turn=="env": # the testers take their turn
            #agent = 'ag_env'
            children = set()
            count = 1
            for gi in self.get_children_gridworlds(debug):
                children.add(gi)
        else: # the environment takes its turn
            for agent in self.ego_agents:
                #agent = 'ego'
                enabled_actions = self.enabled_system_actions(agent)
                # st()
                children = set()
                count = 1
                for ai in enabled_actions:
                    x,y = enabled_actions[ai]
                    ego_agents = [Agent(name=agent.name, x=x,y=y,v=agent.v, goal=agent.goal)]
                    gi = GridWorld(self.lanes, self.width, self.initial_scene,ego_agents=ego_agents, env_agents=self.env_agents)
                    count = count+1
                    children.add(gi)
        return children

def make_gridworld(agentdata,egodata):
    '''Create a gridworld from a list of agents'''
    env_agents = [Agent(name=agent[0], x=agent[1],y=agent[2],v=agent[3], goal=agent[4]) for agent in agentdata]
    gi = GridWorld(2, TRACKLENGTH, [],ego_agents=egodata, env_agents=env_agents, turn="ego")
    return gi

# Debug function:
def print_child_gridworlds(children_list):
    for ci in children_list:
        print(ci.turn + " can play ")
        ci.print_state()
        # print("\n")
