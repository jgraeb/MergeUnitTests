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
from omega.games import gr1
# import tulip
from tulip.interfaces.omega import _grspec_to_automaton
from tulip import spec
from tulip import transys
from tulip.synth import sys_to_spec
import logging

class Spec:
    def __init__(self,sys_vars,init,safety,progress):
        self.variables = sys_vars
        self.init = init
        self.safety = safety
        self.prog = progress


class WinningSet:
    def __init__(self):
        self.spec = None
        self.winning_set = None

    def make_labeled_fsm(self):
        print('Making the abstracted FSM')
        logger = logging.getLogger(__name__)

        # Create a finite transition system
        sys_sws = transys.FTS()
        # add the states
        for i in range(8):
            sys_sws.states.add('s' + str(i)+ '_t')
            sys_sws.states.add('s' + str(i)+ '_s')
        # add the actions
        sys_sws.env_actions |= ['stay_s','merge_s','move_s']
        sys_sws.sys_actions |= ['stay_t','move_t']
        # add the transitions - manually currenty as mapped to abstract states
        sys_sws.transitions.add('s1_s','s1_t', env_actions='stay_s')
        sys_sws.transitions.add('s1_t','s1_s', sys_actions='stay_t')
        sys_sws.transitions.add('s2_s','s2_t', env_actions='stay_s')
        sys_sws.transitions.add('s2_t','s2_s', sys_actions='stay_t')
        sys_sws.transitions.add('s1_t','s2_s', sys_actions='move_t')
        sys_sws.transitions.add('s2_s','s1_t', sys_actions='move_s')
        sys_sws.transitions.add('s3_s','s2_t', env_actions='move_s')
        sys_sws.transitions.add('s2_t','s3_s', sys_actions='move_t')
        sys_sws.transitions.add('s3_s','s4_t', env_actions='merge_s')
        sys_sws.transitions.add('s3_s','s3_t', env_actions='stay_s')
        sys_sws.transitions.add('s3_t','s3_s', sys_actions='stay_t')
        sys_sws.transitions.add('s4_t','s4_s', sys_actions='stay_t')
        sys_sws.transitions.add('s4_s','s4_t', env_actions='stay_s')
        sys_sws.transitions.add('s1_s','s0_t', env_actions='merge_s')
        sys_sws.transitions.add('s0_t','s0_s', sys_actions='stay_t')
        sys_sws.transitions.add('s0_s','s0_t', env_actions='stay_s')
        sys_sws.transitions.add('s1_s','s5_t', env_actions='move_s')
        sys_sws.transitions.add('s5_t','s1_s', sys_actions='move_t')
        sys_sws.transitions.add('s5_s','s5_t', env_actions='stay_s')
        sys_sws.transitions.add('s5_t','s5_s', sys_actions='stay_t')
        sys_sws.transitions.add('s5_s','s6_t', env_actions='move_s')
        sys_sws.transitions.add('s6_t','s5_s', sys_actions='move_t')
        sys_sws.transitions.add('s6_t','s6_s', sys_actions='stay_t')
        sys_sws.transitions.add('s6_s','s6_t', env_actions='stay_s')
        sys_sws.transitions.add('s5_s','s7_t', env_actions='merge_s')
        sys_sws.transitions.add('s7_t','s7_s', sys_actions='stay_t')
        sys_sws.transitions.add('s7_s','s7_t', env_actions='stay_s')
        # specify initial conditions if desired
        # sys_sws.states.initial.add('s1_s')
        # sys_sws.states.initial.add('s2_s')
        # add the atomic propositions
        sys_sws.atomic_propositions.add_from({'goal'})
        sys_sws.states.add('s0_s', ap={'goal'})
        sys_sws.states.add('s0_t', ap={'goal'})
        print(sys_sws)
        sys_sws.save('example_merge_in_front.pdf')
        return sys_sws

    def spec_from_fsm(self, sys_sws):
        ignore_initial = True
        statevar = 'state'
        spec = sys_to_spec(sys_sws, ignore_initial, statevar,bool_states=False, bool_actions=False)
        spec.sys_prog.append('goal')
        print(spec.pretty())
        return spec

    def make_compatible_automaton(self,spec):
        """
        create automaton from spec and abstraction
        @type spec: `tulip.spec.form.GRSpec`
        @type aut: `omega.symbolic.temporal.Automaton`
        """
        # logging.basicConfig(level=logging.WARNING)
        # show = False
        # self.spec = spec.GRSpec(env_vars=self.ego_spec.variables, sys_vars=self.test_spec.variables,
        #             env_init=self.ego_spec.init, sys_init=self.test_spec.init,
        #             env_safety=self.ego_spec.safety, sys_safety=self.test_spec.safety,
        #             env_prog=self.ego_spec.prog, sys_prog=self.test_spec.prog)
        # print(self.spec.pretty())
        aut = _grspec_to_automaton(spec) #`temporal.Automaton` - compiled game with <>[] \/ []<> winning
        return aut


    def find_winning_set(self,spec):
        # interface with TuLiP
        aut = self.make_compatible_automaton(spec)
        z, yij, xijk = gr1.solve_streett_game(aut)
        self.winning_set = z
        return z


    def synthesize_shield(self):
        # create dictionary of allowable actions per state
        # for now manually -> should be from winning set
        shield_dict = dict()
        shield_dict.update({'s0_t':['move']})
        shield_dict.update({'s1_t':['move','stay']})
        shield_dict.update({'s2_t':['stay']})
        shield_dict.update({'s5_t':['move']})
        # print(shield_dict)
        return shield_dict

    def get_winning_set_shield(self):
        fsm = w_set.make_labeled_fsm()
        spec = w_set.spec_from_fsm(fsm)
        winning_set = w_set.find_winning_set(spec)
        shield_dict = w_set.synthesize_shield()
        return shield_dict

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

def simple_test_specs():
    sys_vars = {}
    sys_vars['x'] = (0, 3)
    sys_init = {'x='+str(0)}
    sys_safe = set()
    sys_safe |= {'x=0 -> X((x=0) || (x=1))'}
    sys_safe |= {'x=1 -> X((x=1) || (x=2))'}
    sys_safe |= {'x=2 -> X((x=2) || (x=3))'}
    sys_prog = {'x=3'}
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    tester_vars = {}
    tester_init = set()
    tester_safe = set()
    tester_prog = set()
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    return ego_spec, test_spec

if __name__ == '__main__':
    # testing winning set computation
    # define the specs here
    # ego_spec, test_spec = simple_test_specs()
    # system
    w_set = WinningSet()
    fsm = w_set.make_labeled_fsm()
    spec = w_set.spec_from_fsm(fsm)
    winning_set = w_set.find_winning_set(spec)
    shield_dict = w_set.synthesize_shield()
