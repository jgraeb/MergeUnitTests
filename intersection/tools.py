import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od
from omega.symbolic import temporal as trl
import _pickle as pickle
import os
from omega.games import enumeration as enum
from omega.symbolic import enumeration as sym_enum
from omega.games import gr1
from omega.logic import syntax as stx
from omega.symbolic import fixpoint as fx
from omega.symbolic import fol as _fol
from omega.symbolic import prime as prm
from tulip.interfaces.omega import _grspec_to_automaton, _strategy_to_state_annotated
import logging
import pdb
from tulip import transys, spec, synth

def synthesize_some_controller(aut):
    """Return a controller that implements the spec.
    If no controller exists,
    then raise an `Exception`.
    The returned controller is
    represented as a `networkx` graph.
    """
    z, yij, xijk = gr1.solve_streett_game(aut)
    gr1.make_streett_transducer(
        z, yij, xijk, aut)
    g = enum.action_to_steps(
        aut,
        env='env',
        sys='impl',
        qinit=aut.qinit)
    return g


def create_intersection_from_file(intersectionfile):
    map = od()
    f = open(intersectionfile, 'r')
    lines = f.readlines()
    len_y = len(lines)
    for i,line in enumerate(lines):
        for j,item in enumerate(line):
            if item != '\n':
                map[i,j] = item
    # make dictionary that maps each crosswalk state to a grid cell
    # currenly manual -> TODO crosswalk also automatically from file
    crosswalk = dict()
    start_cw = 1
    end_cw = 5
    y = 2
    for i, num in enumerate(range(2*start_cw,2*(end_cw+1))):
        crosswalk.update({i: (int(np.floor(num/2)), y)})
    # st()
    return map, crosswalk

class WinningSet:
    def __init__(self):
        self.spec = None
        self.winning_set = None

    def set_spec(self, spec):
        self.spec = spec

    def make_compatible_automaton(self,spec):
        """
        create automaton from spec and abstraction
        opt =  tester (test environment needs a controller) or system (system needs a controller)
        @type spec: `tulip.spec.form.GRSpec`
        @type aut: `omega.symbolic.temporal.Automaton`
        """
        aut = _grspec_to_automaton(spec) #`temporal.Automaton` - compiled game with <>[] \/ []<> winning
        return aut

    # Function to check whether state can be in winning set
    # Inputs: Automaton aut, winning set BDD z, and state dictionary state_dict
    # The state dictionary should be as follows: state_dict = {'x': 10, 'y':15}, if the state
    # want to check has values of x=10 and y=15.

    def check_state_in_fp(self, aut, z, state_dict):
        # pdb.set_trace()
        check_bdd = aut.let(state_dict, z)
        if check_bdd == aut.true:
            return True
        elif check_bdd == aut.false:
            return False
        else:
            print("Boolean expression not returned")

    def find_winning_set(self,aut):
        # interface with TuLiP
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

# State given in tuple form: ((x,y), (x1,y1),p)
# Returned in state_dict form (ex): {'x': 2, 'y':3, 'x1': 3, 'y1': 4, 'p': 4}
def convert_tuple2dict(state):
    state_dict = {'y': state[0][0], 'z': state[0][1], 'y1': state[1][0], 'z1': state[1][1],'p': state[2]}
    return state_dict

def check_all_states_in_fp(W, fixpt, aut, sys_st2ver_dict, test_st2ver_dict):
    # winning_set = w_set.find_winning_set(aut)
    print_flg = False
    states_in_winset = []
    states_outside_winset = []
    # for state in sys_st2ver_dict.keys():
    #     state_dict = convert_tuple2dict(state)
    #     check_bdd = W.check_state_in_fp(aut, fixpt, state_dict)
    #     print(state)
    #     print(check_bdd)
    #     if check_bdd:
    #         states_in_winset.append(state)
    #     else:
    #         states_outside_winset.append(state)

    for state in test_st2ver_dict.keys():
        state_dict = convert_tuple2dict(state)
        check_bdd = W.check_state_in_fp(aut, fixpt, state_dict)
        print(state)
        print(check_bdd)
        if check_bdd:
            states_in_winset.append(state)
        else:
            states_outside_winset.append(state)

    return states_in_winset, states_outside_winset

# Design the filter for intersection:
# Checks states that satisfy the assumptions:
# Input: dictionary state
def check_assumptions(state):
    in_W = True # default
    # Ego is on the left turn lane and tester has not yet passed
    if state['y'] == 3:
        ped_not_crossed = lambda state: (state['p'] == 0 or state['p'] == 1 or state['p'] == 2 or state['p'] == 3 or state['p'] == 4 or state['p'] == 5)
        test_car_not_crossed = lambda state: (state['z1'] == 3) and (state['y1'] == 0 or state['y1'] == 1 or state['y1'] == 2 or state['y1'] == 3)
        if ped_not_crossed(state) or test_car_not_crossed(state):
            in_W = False

    # Ego has not reached the intersection and tester car an pedestrian have already passed
    if state['z'] == 4 and not (state['y'] == 4 or state['y'] == 3):
        ped_crossed = lambda state: (state['p'] == 6 or state['p'] == 7 or state['p'] == 8 or state['p'] == 9)
        test_car_crossed = lambda state: (state['z1'] == 3) and (state['y1'] == 4 or state['y1'] == 5 or state['y1'] == 6 or state['y1'] == 7)
        if ped_crossed(state) or test_car_crossed(state):
            in_W = False

    # Collisions:
    collision_ped = lambda state: (state['z'] == 2 and state['y'] == 3 and (state['p'] == 4 or state['p'] == 5))
    collision_car = lambda state: (state['z'] == 3 and state['y'] == 3 and state['y1'] == 3 and state['z1'] == 3)

    if collision_ped(state) or collision_car(state):
        in_W = False
    return in_W

## Check if states are in winning set for receding horizon winning sets:
# w_orig: original winning set from the fixpoint computation
def check_all_states_in_winset(w_orig):
    # winning_set = w_set.find_winning_set(aut)
    states_in_winset = []
    states_outside_winset = []
    # print('Filtered WS')

    # x2 < x1, since x2 is a second tester
    for state in w_orig:
        state_dict = convert_tuple2dict(state)
        flg = check_assumptions(state_dict)
        if flg:
            states_in_winset.append(state)
        else:
            states_outside_winset.append(state)
        # print(state)
        # print(flg)
    return states_in_winset, states_outside_winset

# Least fixpoint computation:
# General comment: Write specifications as a closed system and then open it
#
def descendants(source, constrain, aut, future=True):
    """Existential descendants of `source` in `constrain`.
    @param future: if `True`, then apply an image operation
        before starting the least fixpoint computation
    """
    if future:
        q = ee_image(source, aut)
    else:
        q = source
    qold = None
    while q != qold:
        post = ee_image(q, aut)
        qold = q
        q |= post
        q &= constrain
    # assert that it is a state predicate
    return q

def desc_operator():
    while q != qold:
       post = ee_image(q, aut)
       qold = q
       q |= post
       q &= constrain

def ee_image(source, aut):
    """Existential image."""
    u = aut.action[SYS]
    qvars = aut.varlist['env'] + aut.varlist['sys']
    u = aut.exist(qvars, u & source)
    u = prm.unprime(u, aut)
    return u
