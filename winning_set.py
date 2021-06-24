#############################################################################
#                                                                           #
# Winning Set Analysis for Compositional Testing                            #
# Josefine Graebener, Apurva Badithela                                      #
# Caltech, May 2021                                                         #
#                                                                           #
#############################################################################
from random import choice
# from mcts import MCTS, Node
import numpy as np
from scene import Scene
from agent import Agent
from map import Map
import _pickle as pickle
import os
from copy import deepcopy
from ipdb import set_trace as st
import omega
# import tulip
from tulip.interfaces.omega import _grspec_to_automaton
from tulip import spec
import logging

class Spec:
    def __init__(self,sys_vars,init,safety,progress):
        self.variables = sys_vars
        self.init = init
        self.safety = safety
        self.prog = progress


class WinningSet:
    def __init__(self,test_spec, ego_spec):
        self.test_spec = test_spec
        self.ego_spec = ego_spec
        self.spec = None
        self.winning_set = None

    def make_compatible_automaton(self):
        """
        create automaton from spec and abstraction
        @type spec: `tulip.spec.form.GRSpec`
        @type aut: `omega.symbolic.temporal.Automaton`
        """
        logging.basicConfig(level=logging.WARNING)
        show = False
        self.spec = spec.GRSpec(env_vars=self.ego_spec.variables, sys_vars=self.test_spec.variables,
                    env_init=self.ego_spec.init, sys_init=self.test_spec.init,
                    env_safety=self.ego_spec.safety, sys_safety=self.test_spec.safety,
                    env_prog=self.ego_spec.prog, sys_prog=self.test_spec.prog)
        print(self.spec.pretty())
        aut = _grspec_to_automaton(self.spec) #`temporal.Automaton` - compiled game with <>[] \/ []<> winning
        return aut


    def find_winning_set(self):
        # interface with TuLiP
        aut = self.make_compatible_automaton()
        z, yij, xijk = omega.solve_streett_game(aut)
        self.winning_set = z


    def synthesize_shield(self):
        # create dictionary of allowable actions per state
        pass

def specs_for_entire_track(tracklength):
    sys_vars = {}
    sys_vars['x'] = (0, tracklength) # 0: system is much behind the second tester car, 6: system is much further than the first tester car
    sys_vars['y'] = (0,1)
    sys_init = {'x='+str(0), 'y='+str(0)}
    sys_prog = {'y=1'} # Eventually, the system should merge
    sys_safe = set()
    sys_safe |= {'!(x1=x && y1=y)'}
    sys_safe |= {'!(x2=x && y2=y)'}
    for ii in range(1,tracklength-1):
        sys_safe |= {'(x='+str(ii)+' && y=0) -> X((x='+str(ii+1)+' && y=0)||(x='+str(ii)+' && y=0)|| (x='+str(ii+1)+' && y=1))'}
        sys_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=0))'}
    sys_safe |= {'x=0 -> X(x=0 && x=1)'}
    sys_safe |= {'x=9 -> X(x=9 && x=10)'}
    sys_safe |= {'x=10 -> X(x=10)'}
    # testers
    tester_vars = {}
    tester_vars['x1'] = (1,tracklength)
    tester_vars['y1'] = (0,1)
    tester_vars['x2'] = (0,tracklength-1)
    tester_vars['y2'] = (0,1)
    tester_init = {'x1='+str(1), 'y1='+str(1), 'x2='+str(0), 'y2='+str(1)}
    tester_prog = set()
    for ki in range(1,tracklength-1):
        tester_prog |= {'((x='+str(ki+1)+') && (x1='+str(ki+2)+') && (x2='+str(ki)+')) && (y=1 && y1=1 && y2=1)'}
    tester_safe = set()
    tester_safe |= {'!(x1=x2 && y1=y2)'}
    tester_safe |= {'!(x1=x && y1=y)'}
    tester_safe |= {'!(x2=x && y2=y)'}
    tester_safe |= {'!(y1=1) && !(y2=1)'} # testers stay in bottom lane
    for ii in range(1,tracklength-1):
        tester_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1)|| (x1='+str(ii+1)+' && y1=2))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2)|| (x2='+str(ii+1)+' && y2=1))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1)|| (x2='+str(ii+1)+' && y2=2))'}
        tester_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2)|| (x1='+str(ii+1)+' && y1=1))'}
    tester_safe |= {'!(x1='+str(tracklength)+' && x2=0)'}
    tester_safe |= {'(x2='+str(tracklength-1)+' -> X(x2='+str(tracklength-1)+')'}
    tester_safe |= {'(x1='+str(tracklength)+' -> X(x1='+str(tracklength)+')'}
    tester_safe |= {'(x1='+str(tracklength-1)+' -> X(x1='+str(tracklength-1)+' || x1='+str(tracklength)+')'}
    tester_safe |= {'(x2=0 -> X(x2=0 || x2=1)'}
    #
    for xi in range(0,tracklength+1):
        for x1i in range(1,tracklength):
            for x2i in range(0,x1i):
                other_st_i = '((x = '+str(xi)+') && (x1='+str(x1i)+')&&(x2='+str(x2i)+'))'
                sys_safe |= {'((y=1) && '+other_st_i+')-> X('+other_st_i+'&& (y=1))'}
                tester_safe |= {'((y=1) && '+other_st_i+')-> X('+other_st_i+'&& (y=1))'}
    #
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    return ego_spec, test_spec

if __name__ == '__main__':
    # testing winning set computation
    # define the specs here
    ego_spec, test_spec = specs_for_entire_track(10)
    # system
    w_set = WinningSet(test_spec,ego_spec)
    w_set.find_winning_set()
    st()
