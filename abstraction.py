#############################################################################
#                                                                           #
# Abstraction for Compositional Testing                                     #
# Josefine Graebener, Apurva Badithela                                      #
# Caltech, May 2021                                                         #
#                                                                           #
#############################################################################
import numpy as np
# from ipdb import set_trace as st
import omega
import tulip
import pdb

class State:
    def __init__(self, sys_pos, t1_pos, t2_pos):
        self.sys = sys_pos
        self.t1 = t1_pos
        self.t2 = t2_pos

    def pretty(self):
        return ((self.sys),(self.t1),(self.t2))

class Transition:
    def __init__(self, start, end, action, type):
        self.start = start
        self.end = end
        self.action = action
        self.type = type

class Abstraction:
    def __init__(self,lanes, length, sys_actions, tester_actions):
        self.lanes = lanes # number of lanes
        self.length = length # number of cells in a lane odd number
        self.sys_actions = sys_actions
        self.tester_actions = tester_actions
        self.states = []
        self.transitions = []


    def setup_abstraction(self):
        ys = np.arange(0,self.lanes)
        xs = np.arange(0,self.length)-np.floor(self.length/2)
        # keep ego in (0,0)
        # enumerate all possible state combinations of system and testers
        # add all different conbinations of testers in bottom lane
        for x1 in xs:
            for x2 in xs:
                if x2>x1:
                    self.add_state((0,0),(x1,1),(x2,1))
        # add winning state with TST in top lane (relatively)
        self.add_state((0,0),(-1,0),(1,0))
        print('States:')
        for state in self.states:
            print(state.sys, state.t1, state.t2)
        # add the transitions (tester actions normally, system actions move testers back relatively)
        # go through tester tester_actions
        for state in self.states:
            t1 = state.t1
            t2 = state.t2
            sys = state.sys
            for action1 in self.tester_actions:
                new_t2 = tuple([sum(bb) for bb in zip(self.tester_actions[action1], t2)])
                for action2 in self.tester_actions:
                    new_t1 = tuple([sum(bb) for bb in zip(self.tester_actions[action2], t1)])
                    if new_t1 != new_t2 and new_t1!= sys and new_t2!= sys:
                        newstate = State((0,0),new_t1,new_t2)
                        self.add_transition(state,newstate,[action1,action2],'tester')
        # add system transitions
        for state in self.states:
            t1 = state.t1
            t2 = state.t2
            sys = state.sys
            for action in self.sys_actions:
                newsys, sys , new_t1, new_t2 = self.map_sys_action_2_testers(state,action)
                # if action == 'mergeR':
                #     st()
                if sys != new_t1 and sys != new_t2 and newsys != t1 and newsys != t2:
                    newstate = State(sys,new_t1, new_t2)
                    self.add_transition(state,newstate,action,'sys')

        print('Transitions:')
        for transition in self.transitions:
            print('{0} to {1}, actions: {2}, type: {3}'.format(transition.start.pretty(),transition.end.pretty(),transition.action, transition.type))


    def map_sys_action_2_testers(self, state, action):
        t1 = state.t1
        t2 = state.t2
        sys = state.sys
        new_sys = tuple([sum(bb) for bb in zip(self.sys_actions[action], sys)])
        new_t1 = tuple(map(lambda i, j: i - j, t1, new_sys))
        new_t2 = tuple(map(lambda i, j: i - j, t2, new_sys))
        # if action == 'mergeR':
        #     st()
        return new_sys, sys, new_t1, new_t2

    def add_transition(self,start,end,action, type):
        self.transitions.append(Transition(start,end,action,type))

    def add_state(self, sys, t1, t2):
        self.states.append(State(sys,t1,t2))

    def make_gr_spec(self):
        specs = spec_pipeline(self)
        st()



if __name__ == '__main__':
    # actions passed from gridworld
    sys_actions = {'move': (1,0), 'stay': (0,0), 'mergeR': (1,1)} # possible actions for ego

     # possible actions for each tester

    abs = Abstraction(2,3,sys_actions,tester_actions)
    abs.setup_abstraction()
    pdb.set_trace()
