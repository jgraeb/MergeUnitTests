#############################################################################
#                                                                           #
# Winning Set Analysis for Compositional Testing                            #
# Josefine Graebener, Apurva Badithela                                      #
# Caltech, May 2021                                                         #
#                                                                           #
#############################################################################
from __future__ import print_function
from random import choice
# from mcts import MCTS, Node
import numpy as np
from tulip.spec import form
from scene import Scene
from agent import Agent
from map import Map
import networkx as nx
from omega.symbolic import temporal as trl
import _pickle as pickle
import os
from omega.games import enumeration as enum
from omega.symbolic import enumeration as sym_enum
from copy import deepcopy
import ipdb
from omega.games import gr1
from omega.logic import syntax as stx
from omega.symbolic import fixpoint as fx
from omega.symbolic import fol as _fol
from omega.symbolic import prime as prm
from tulip.interfaces.omega import _grspec_to_automaton, _strategy_to_state_annotated
from tulip import spec
from tulip import transys
from tulip.synth import sys_to_spec
import logging
from ipdb import set_trace as st
from tulip import transys, spec, synth

class Spec:
    def __init__(self,sys_vars,init,safety,progress):
        self.variables = sys_vars
        self.init = init
        self.safety = safety
        self.prog = progress

# Get GRSpec:
# Standard form controller synthesis of GR(1) specifications:
# env_init & []env_safe & []<>env_prog --> sys_init & []sys_safe & []<>sys_prog
def make_grspec(sys_spec, env_spec):
    env_vars = env_spec.variables
    sys_vars = sys_spec.variables
    env_init = env_spec.init
    sys_init = sys_spec.init
    env_safe = env_spec.safety
    sys_safe = sys_spec.safety
    env_prog = env_spec.prog
    sys_prog = sys_spec.prog
    return spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)

class WinningSet:
    def __init__(self):
        self.spec = None
        self.winning_set = None

    def set_spec(self, spec):
        self.spec = spec

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
        # sys_sws.transitions.add('s1_s','s1_t', env_actions='stay_s')
        sys_sws.transitions.add('s1_t','s1_s', sys_actions='stay_t')
        # sys_sws.transitions.add('s2_s','s2_t', env_actions='stay_s')
        sys_sws.transitions.add('s2_t','s2_s', sys_actions='stay_t')
        sys_sws.transitions.add('s1_t','s2_s', sys_actions='move_t')
        sys_sws.transitions.add('s2_s','s1_t', env_actions='move_s')
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
        # sys_sws.transitions.add('s1_s','s5_t', env_actions='move_s')
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
        # spec.sys_prog.append('goal')
        spec.env_prog.append('goal')
        print(spec.pretty())
        return spec

    def make_compatible_automaton(self,spec):
        """
        create automaton from spec and abstraction
        opt =  tester (test environment needs a controller) or system (system needs a controller)
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

    # Function to check whether state can be in winning set
    # Inputs: Automaton aut, winning set BDD z, and state dictionary state_dict
    # The state dictionary should be as follows: state_dict = {'x': 10, 'y':15}, if the state
    # want to check has values of x=10 and y=15.

    def check_state_in_winset(self, aut, z, state_dict):
       # all identifiers are unprimed ?
       # avoid priming constants
       # (no primed identifiers are declared for constants)
        # state_dict = {state: stx.prime(state)}

        # Debugging scripts have to be removed
        # support = aut.support(z)
        # all identifiers are unprimed ?
        # assert not any(stx.isprimed(name) for name in support), support
        # avoid priming constants
        # (no primed identifiers are declared for constants)
        #vrs = {name for name in support if prm.is_variable(name, aut)}
        # state_dict = {var: stx.prime(var) for var in vrs}

        # test
        # vrs = {'x=3'}
        # state_dict = {var: stx.prime(var) for var in vrs}
        # st()
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
        # type_hint_expr = aut.type_hint_for(['X1', 'X2', 'X3'])
        # care = aut.add_expr(type_hint_expr)
        # expr = aut.to_expr(z, care=care, show_dom=True)
        # type_hint_expr = aut.type_hint_for(['x', 'y'])
        # care = aut.add_expr(type_hint_expr)
        # expr = aut.to_expr(z, care=None, show_dom=True)
        # expr = aut.to_expr(z)
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


# Testing winning set specifications for automaton:
def example_win_set3():
    """Return a temporal logic spec in the GR(1) fragment."""
    aut = trl.Automaton()
    aut.declare_variables(x=(1, 5), y=(1, 5))
    aut.varlist.update(env=['y'], sys=['x'])
    aut.init['env'] = 'y = 3'
    aut.init['sys'] = 'x = 1'
    # x \in 1..2 -> x \in  [1,2), x' \in [1,5)
    # Add safety:
    aut.action['sys'] = r'''
        /\ (\/ (x = 1 /\ x' \in 1..4)
            \/ (x = 2 /\ (x' \in 1..3 \/ x' = 5))
            \/ (x = 3 /\ (x' \in 1..5))
            \/ (x = 4 /\ (x' = 1 \/ x' \in 3..5))
            \/ (x = 5 /\ (x' \in 2..5)))
        /\ (\/ (x = 1 /\ y \in 2..5)
            \/ (x = 2 /\ (y = 1 \/ y \in 3..5))
            \/ (x = 3 /\ (y \in 1..2 \/ y \in 4..5))
            \/ (x = 4 /\ (y \in 1..3 \/ y = 5))
            \/ (x = 5 /\ (y \in 1..4)))
        '''

    aut.action['env'] = r'''
        \/ (y = 1 /\ y' = 1)
        \/ (y = 2 /\ (y' \in 2..3))
        \/ (y = 3 /\ (y' = 2\/ y' = 4))
        \/ (y = 4/\ (y' \in 3..4))
        \/ (y = 5/\ (y' = 5))
        '''
    aut.win['<>[]'] = aut.bdds_from('x = 5')
    aut.win['[]<>'] = aut.bdds_from('x = 5')
    aut.qinit = r'\E \A'
    aut.moore = True
    aut.plus_one = True

    return aut

# Testing winning set specifications:
def example_win_set2():
    sys_vars = {}
    sys_vars['X1'] = 'boolean'
    sys_vars['X2'] = 'boolean'
    sys_vars['X3'] = 'boolean'
    sys_vars['X4'] = 'boolean'
    sys_vars['X5'] = 'boolean'
    sys_init = {'X1'}
    sys_safe = {
    'X1 -> X (X3 || X4 || X2)',
    'X2 -> X (X1 || X5 || X4)',
    'X3 -> X (X1 || X4 || X2 ||X5)',
    'X4 -> X (X3 || X1 || X5)',
    'X5 -> X (X4 || X2 || X3)',
    }
    sys_prog = set()
    sys_prog |= {'X5'}

    env_vars = {}
    env_vars['T1'] = 'boolean'
    env_vars['T2'] = 'boolean'
    env_vars['T3'] = 'boolean'
    env_vars['T4'] = 'boolean'
    env_vars['T5'] = 'boolean'

    env_init = {'T2'}
    env_safe = {
    'T1 -> X (T1)',
    'T2 -> X (T2 || T3)',
    'T3 -> X (T2|| T4)',
    'T4 -> X (T3 || T4)',
    'T5 -> X (T5)',
    }
    env_prog = {}

    # @specs_setup_section_end@

    sys_safe |= {'!(T1 && X1)'}
    sys_safe |= {'!(T2 && X2)'}
    sys_safe |= {'!(T3 && X3)'}
    sys_safe |= {'!(T4 && X4)'}
    sys_safe |= {'!(T5 && X5)'}
    # @specs_create_section@
    # Create the specification
    specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                        env_safe, sys_safe, env_prog, sys_prog)

    # Controller synthesis
    #
    # At this point we can synthesize the controller using one of the available
    # methods.
    #
    # @synthesize@
    # Moore machines
    # controller reads `env_vars, sys_vars`, but not next `env_vars` values
    specs.moore = True
    # synthesizer should find initial system values that satisfy
    # `env_init /\ sys_init` and work, for every environment variable
    # initial values that satisfy `env_init`.
    specs.qinit = r'\E \A'
    return specs


# Testing winning set specifications:
def example_win_set():
    sys_vars = {}
    sys_vars['x'] = (1, 5)
    # sys_vars['goal_reach'] = 'boolean'
    sys_init = {'x=1'}
    sys_safe = {
    'x=1 -> X(x=3 || x=4 || x=2 || x=1)',
    'x=2 -> X(x=2 || x=3 || x=5)',
    'x=3 -> X(x=4 || x=2 || x=3 ||x=5)',
    'x=4 -> X(x=3 || x=5 || x=4)',
    'x=5 -> X(x=5)',
    }
    sys_prog = set()
    # sys_safe |= {'(x=5)<-> goal_reach'}
    sys_prog |= {'x=5'}

    env_vars = {}
    env_vars['t'] = (1, 5)

    env_init = {'t=3'}
    env_safe = {
    't=1 -> X(t=1)',
    't=2 -> X(t=3)',
    't=3 -> X(t=2|| t=4)',
    't=4 -> X(t=3)',
    't=5 -> X(t=5)',
    }
    env_prog = {}

    # @specs_setup_section_end@

    sys_safe |= {'!(t=1 && x=1)'}
    sys_safe |= {'!(t=2 && x=2)'}
    sys_safe |= {'!(t=3 && x=3)'}
    sys_safe |= {'!(t=4 && x=4)'}
    sys_safe |= {'!(t=5 && x=5)'}
    # @specs_create_section@
    # Create the specification
    specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                        env_safe, sys_safe, env_prog, sys_prog)
    # pdb.set_trace()
    ctrl = synth.synthesize(specs)
    assert ctrl is not None, 'unrealizable'
    # At this point we can synthesize the controller using one of the available
    # methods.
    #
    # @synthesize@
    # Moore machines
    # controller reads `env_vars, sys_vars`, but not next `env_vars` values
    specs.moore = True
    # synthesizer should find initial system values that satisfy
    # `env_init /\ sys_init` and work, for every environment variable
    # initial values that satisfy `env_init`.
    specs.qinit = r'\E \A'
    return specs

def test_spec():
    sys_vars = {}
    sys_vars['x'] = (1, 3)
    sys_vars['y'] = (1,2)
    sys_init = {'x='+str(1), 'y='+str(1)}
    sys_prog = {'y=2'} # Eventually merge
    sys_safe = set()
    # Dynamics
    sys_safe |= {'(x=1 && y=1) -> X((x=1 && y=1) && (x=2 && y=1) && (x=2 && y=2))'}
    sys_safe |= {'(x=2 && y=1) -> X((x=2 && y=1) && (x=3 && y=1) && (x=3 && y=2))'}
    sys_safe |= {'(x=3 && y=1) -> X(x=3 && y=1)'}
    sys_safe |= {'(x=1 && y=2) -> X((x=1 && y=2) && (x=2 && y=2) && (x=2 && y=1))'}
    sys_safe |= {'(x=2 && y=2) -> X((x=2 && y=2) && (x=3 && y=2) && (x=3 && y=1))'}
    sys_safe |= {'(x=3 && y=2) -> X(x=3 && y=2)'}
    # Testers
    tester_vars = {}
    tester_vars['x1'] = (1,3)
    tester_vars['y1'] = (1,2)
    tester_vars['x2'] = (1,3)
    tester_vars['y2'] = (1,2)
    tester_init = {'x1='+str(2), 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    tester_prog = set() # for now everything is ok
    tester_prog = {'(y1=2 && y2=2)'}
    tester_safe = set()
    # no collision
    for ii in range(1,3+1):
        for jj in range(1,2+1):
            sys_safe |= {'!(x='+str(ii)+' && x1 ='+str(ii)+' && y= '+str(jj)+ ' && y1 = '+str(jj)+')'}
            sys_safe |= {'!(x='+str(ii)+' && x2 ='+str(ii)+' && y= '+str(jj)+ ' && y2 = '+str(jj)+')'}
            tester_safe |= {'!(x1='+str(ii)+' && x2 ='+str(ii)+' && y1= '+str(jj)+ ' && y2 = '+str(jj)+')'}
    # Dynamics
    tester_safe |= {'(x1=1 && y1=2) -> X((x1=1 && y1=2) && (x1=2 && y1=2))'}
    tester_safe |= {'(x1=2 && y1=2) -> X((x1=2 && y1=2) && (x1=3 && y1=2))'}
    tester_safe |= {'(x1=3 && y1=2) -> X(x1=3 && y1=2)'}
    tester_safe |= {'(x2=1 && y2=2) -> X((x2=1 && y2=2) && (x2=2 && y2=2))'}
    tester_safe |= {'(x2=2 && y2=2) -> X((x2=2 && y2=2) && (x2=3 && y2=2))'}
    tester_safe |= {'(x2=3 && y2=2) -> X(x2=3 && y2=2)'}
    # Synthesize specs
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    # st()

    return ego_spec, test_spec

def spec_merge_in_front():
    sys_vars = {}
    sys_vars['x'] = (1, 3)
    sys_vars['y'] = (1,2)
    sys_init = {'x='+str(1), 'y='+str(1)}
    sys_prog = {'y=2'} # Eventually merge
    sys_safe = set()
    # Dynamics
    sys_safe |= {'(x=1 && y=1) -> X((x=1 && y=1) && (x=2 && y=1) && (x=2 && y=2))'}
    sys_safe |= {'(x=2 && y=1) -> X((x=2 && y=1) && (x=3 && y=1) && (x=3 && y=2))'}
    sys_safe |= {'(x=3 && y=1) -> X(x=3 && y=1)'}
    sys_safe |= {'(x=1 && y=2) -> X((x=1 && y=2) && (x=2 && y=2) && (x=2 && y=1))'}
    sys_safe |= {'(x=2 && y=2) -> X((x=2 && y=2) && (x=3 && y=2) && (x=3 && y=1))'}
    sys_safe |= {'(x=3 && y=2) -> X(x=3 && y=2)'}
    # Testers
    tester_vars = {}
    tester_vars['x1'] = (1,3)
    # tester_vars['y1'] = (1,2)
    tester_init = {'x1='+str(1)}#, 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    tester_prog = set() # for now everything is ok
    # tester_prog = {'(y=2)'}
    tester_safe = set()
    # no collision
    for ii in range(1,3+1):
        for jj in range(1,2+1):
            sys_safe |= {'!(x='+str(ii)+' && x1 ='+str(ii)+' && y= '+str(jj)+')'}
            tester_safe |= {'!(x='+str(ii)+' && x1 ='+str(ii)+' && y= '+str(jj)+')'}
            # sys_safe |= {'!(x='+str(ii)+' && x2 ='+str(ii)+' && y= '+str(jj)+ ' && y2 = '+str(jj)+')'}
            # tester_safe |= {'!(x1='+str(ii)+' && x2 ='+str(ii)+' && y1= '+str(jj)+ ' && y2 = '+str(jj)+')'}
    # Dynamics
    tester_safe |= {'(x1=1) -> X((x1=1) && (x1=2))'}
    tester_safe |= {'(x1=2) -> X((x1=2) && (x1=3))'}
    tester_safe |= {'(x1=3) -> X(x1=3)'}
    # tester_safe |= {'(x2=1 && y2=2) -> X((x2=1 && y2=2) && (x2=2 && y2=2))'}
    # tester_safe |= {'(x2=2 && y2=2) -> X((x2=2 && y2=2) && (x2=3 && y2=2))'}
    # tester_safe |= {'(x2=3 && y2=2) -> X(x2=3 && y2=2)'}
    # Synthesize specs
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    # st()
    return ego_spec, test_spec

# System and environment have been switched:
def specs_for_entire_track_vMod(tracklength):
    # st()
    sys_vars = {}
    sys_vars['x'] = (1, tracklength) # 0: system is much behind the second tester car, 6: system is much further than the first tester car
    sys_vars['y'] = (1,2)
    sys_init = {'x='+str(1), 'y='+str(1)}
    sys_prog = {'y=2'} # Eventually, the system should merge
    sys_safe = set()

    # Dynamics for merging into adjacent track:
    for ii in range(1,tracklength):
        sys_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=2))'}
        sys_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2)|| (x='+str(ii+1)+' && y=1))'}
    sys_safe |= {'(x='+str(tracklength)+' && y=1) -> X(x='+str(tracklength)+' && y=1)'}
    sys_safe |= {'(x='+str(tracklength)+' && y=2) -> X(x='+str(tracklength)+' && y=2)'}
    # sys_safe |= {'x=0 -> X(x=0 && x=1)'}
    # sys_safe |= {'x='+str(tracklength-1)+' -> X(x='+str(tracklength-1)+' && x='+str(tracklength)+')'}
    # sys_safe |= {'x='+str(tracklength)+'-> X(x='+str(tracklength)+')'}

    # testers
    tester_vars = {}
    tester_vars['x1'] = (2,tracklength)
    tester_vars['y1'] = (1,2)
    tester_vars['x2'] = (1,tracklength-1)
    tester_vars['y2'] = (1,2)
    tester_init = {'x1='+str(2), 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    tester_prog = set()
    # for ki in range(1,tracklength-1):
    #     tester_prog |= {'((x='+str(ki+1)+') && (x1='+str(ki+2)+') && (x2='+str(ki)+')) && (y=2 && y1=2 && y2=2)'}
    # tester_prog |= {'(y=2 && y1=2 && y2=2)'}
    tester_safe = set()

    # No collision with other vehicles:
    for yi in range(1,3):
        for xi in range(1, tracklength+1):
            if xi!= tracklength:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                tester_safe |= {'!(x='+str(xi)+' && x2 ='+str(xi)+' && y= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                sys_safe |= {'!(x='+str(xi)+' && x2 ='+str(xi)+' && y= '+str(yi)+ ' && y2 = '+str(yi)+')'}

            if xi != 1:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                tester_safe |= {'!(x='+str(xi)+' && x1 ='+str(xi)+' && y= '+str(yi)+ ' && y1 = '+str(yi)+')'}
                sys_safe |= {'!(x='+str(xi)+' && x1 ='+str(xi)+' && y= '+str(yi)+ ' && y1 = '+str(yi)+')'}

    tester_safe |= {'!(y1=1) && !(y2=1)'} # testers stay in bottom lane

    # Tester dynamics
    for ii in range(1,tracklength):
        # tester_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1))'}#|| (x1='+str(ii+1)+' && y1=2))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2))'}#|| (x2='+str(ii+1)+' && y2=1))'}
        # tester_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1))'}#|| (x2='+str(ii+1)+' && y2=2))'}
        tester_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2))'}#|| (x1='+str(ii+1)+' && y1=1))'}
    # tester_safe |= {'!(x1='+str(tracklength)+' && x2=0)'}
    tester_safe |= {'(x2='+str(tracklength-1)+' && y1=2) -> X(x2='+str(tracklength-1)+' && y1=2)'}
    tester_safe |= {'(x1='+str(tracklength)+' && y1=2) -> X(x1='+str(tracklength)+' && y1=2)'}
    # tester_safe |= {'(x1='+str(tracklength-1)+') -> X(x1='+str(tracklength-1)+' || x1='+str(tracklength)+')'}
    # tester_safe |= {'(x2=0) -> X(x2=0 || x2=1)'}

    # Terminal conditions: Once car merges, it remains merged.
    # for xi in range(0,tracklength+1):
    #     for x1i in range(1,tracklength):
    #         for x2i in range(0,x1i):
    #             other_st_i = '(x = '+str(xi)+') && (x1='+str(x1i)+')&&(x2='+str(x2i)+')'
    #             sys_safe |= {'((y=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2))'}
    #             tester_safe |= {'((y=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2))'}

    # Synthesize specs
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    # st()

    return ego_spec, test_spec

# System and environment have been switched:
def specs_for_entire_track(tracklength):
    # st()
    sys_vars = {}
    sys_vars['x'] = (0, tracklength) # 0: system is much behind the second tester car, 6: system is much further than the first tester car
    sys_vars['y'] = (1,2)
    sys_init = {'x='+str(0), 'y='+str(1)}
    sys_prog = {'y=2'} # Eventually, the system should merge
    sys_safe = set()

    # Dynamics for merging into adjacent track:
    for ii in range(1,tracklength-1):
        sys_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=2))'}
        sys_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2)|| (x='+str(ii+1)+' && y=1))'}
    sys_safe |= {'x=0 -> X(x=0 && x=1)'}
    sys_safe |= {'x='+str(tracklength-1)+' -> X(x='+str(tracklength-1)+' && x='+str(tracklength)+')'}
    sys_safe |= {'x='+str(tracklength)+'-> X(x='+str(tracklength)+')'}

    # testers
    tester_vars = {}
    tester_vars['x1'] = (1,tracklength)
    tester_vars['y1'] = (1,2)
    tester_vars['x2'] = (0,tracklength-1)
    tester_vars['y2'] = (1,2)
    tester_init = {'x1='+str(1), 'y1='+str(2), 'x2='+str(0), 'y2='+str(2)}
    tester_prog = set()
    for ki in range(1,tracklength-1):
        tester_prog |= {'((x='+str(ki+1)+') && (x1='+str(ki+2)+') && (x2='+str(ki)+')) && (y=2 && y1=2 && y2=2)'}
    tester_prog |= {'(y=2 && y1=2 && y2=2)'}
    tester_safe = set()

    # No collision with other vehicles:
    for yi in range(1,3):
        for xi in range(0, tracklength+1):
            if xi!= tracklength:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                tester_safe |= {'!(x='+str(xi)+' && x2 ='+str(xi)+' && y= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                sys_safe |= {'!(x='+str(xi)+' && x2 ='+str(xi)+' && y= '+str(yi)+ ' && y2 = '+str(yi)+')'}

            if xi != 0:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                tester_safe |= {'!(x='+str(xi)+' && x1 ='+str(xi)+' && y= '+str(yi)+ ' && y1 = '+str(yi)+')'}
                sys_safe |= {'!(x='+str(xi)+' && x1 ='+str(xi)+' && y= '+str(yi)+ ' && y1 = '+str(yi)+')'}

    tester_safe |= {'!(y1=1) && !(y2=1)'} # testers stay in bottom lane

    # Tester dynamics
    for ii in range(1,tracklength-1):
        tester_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1)|| (x1='+str(ii+1)+' && y1=2))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2)|| (x2='+str(ii+1)+' && y2=1))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1)|| (x2='+str(ii+1)+' && y2=2))'}
        tester_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2)|| (x1='+str(ii+1)+' && y1=1))'}
    tester_safe |= {'!(x1='+str(tracklength)+' && x2=0)'}
    tester_safe |= {'(x2='+str(tracklength-1)+') -> X(x2='+str(tracklength-1)+')'}
    tester_safe |= {'(x1='+str(tracklength)+') -> X(x1='+str(tracklength)+')'}
    tester_safe |= {'(x1='+str(tracklength-1)+') -> X(x1='+str(tracklength-1)+' || x1='+str(tracklength)+')'}
    tester_safe |= {'(x2=0) -> X(x2=0 || x2=1)'}

    # Terminal conditions: Once car merges, it remains merged.
    for xi in range(0,tracklength+1):
        for x1i in range(1,tracklength):
            for x2i in range(0,x1i):
                other_st_i = '(x = '+str(xi)+') && (x1='+str(x1i)+')&&(x2='+str(x2i)+')'
                sys_safe |= {'((y=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2))'}
                tester_safe |= {'((y=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2))'}

    # Synthesize specs
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    # st()

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

def synthesize_some_controller(aut):
    """Return a controller that implements the spec.
    If no controller exists, then raise an `Exception`.
    The returned controller is represented as a `networkx` graph.
    """
    z, yij, xijk = gr1.solve_streett_game(aut)
    gr1.make_streett_transducer(z, yij, xijk, aut)
    g = enum.action_to_steps(
        aut, env='env', sys='impl', qinit=aut.qinit)
    return g

# BDD winning set: u
# Automaton: aut
def print_expr(u, aut):
    """BDD to expr in case of int and bool vars."""
    # currently, `to_expr` works with only
    # integer-valued variables
    aut.declare_constants(tmp=(0, 1))
    s = '\E z:  ({u} /\ (z <=> (tmp = 1)) )'.format(u=u)
    v = aut.to_bdd(s)
    s = aut.type_hint_for(['x', 'tmp'])
    pdb.set_trace()
    care = aut.to_bdd(s)
    expr = aut.to_expr(v, care)
    print(expr)

# Convert bools:
def convert_bools(aut):
    aut.declare_variables(X1_as_int=(0, 1))
    aut.declare_variables(X2_as_int=(0, 1))
    aut.declare_variables(X3_as_int=(0, 1))
    aut.declare_variables(X4_as_int=(0, 1))
    aut.declare_variables(X5_as_int=(0, 1))

    aut.declare_variables(T1_as_int=(0, 1))
    aut.declare_variables(T2_as_int=(0, 1))
    aut.declare_variables(T3_as_int=(0, 1))
    aut.declare_variables(T4_as_int=(0, 1))
    aut.declare_variables(T5_as_int=(0, 1))

    y_X1 = aut.add_expr("X1")
    y_X2 = aut.add_expr("X2")
    y_X3 = aut.add_expr("X3")
    y_X4 = aut.add_expr("X4")
    y_X5 = aut.add_expr("X5")

    y_T1 = aut.add_expr("T1")
    y_T2 = aut.add_expr("T2")
    y_T3 = aut.add_expr("T3")
    y_T4 = aut.add_expr("T4")
    y_T5 = aut.add_expr("T5")

    x1_over_int = aut.add_expr(rf'\E X1:  (X1 <=> (X1_as_int = 1)) /\ {y_X1}')
    x2_over_int = aut.add_expr(rf'\E X2:  (X2 <=> (X2_as_int = 1)) /\ {y_X2}')
    x3_over_int = aut.add_expr(rf'\E X3:  (X3 <=> (X3_as_int = 1)) /\ {y_X3}')
    x4_over_int = aut.add_expr(rf'\E X4:  (X4 <=> (X4_as_int = 1)) /\ {y_X4}')
    x5_over_int = aut.add_expr(rf'\E X5:  (X5 <=> (X5_as_int = 1)) /\ {y_X5}')

    t1_over_int = aut.add_expr(rf'\E T1:  (T1 <=> (T1_as_int = 1)) /\ {y_T1}')
    t2_over_int = aut.add_expr(rf'\E T2:  (T2 <=> (T2_as_int = 1)) /\ {y_T2}')
    t3_over_int = aut.add_expr(rf'\E T3:  (T3 <=> (T3_as_int = 1)) /\ {y_T3}')
    t4_over_int = aut.add_expr(rf'\E T4:  (T4 <=> (T4_as_int = 1)) /\ {y_T4}')
    t5_over_int = aut.add_expr(rf'\E T5:  (T5 <=> (T5_as_int = 1)) /\ {y_T5}')

    ex_x1 = aut.to_expr(x1_over_int)
    ex_x2 = aut.to_expr(x2_over_int)
    ex_x3 = aut.to_expr(x3_over_int)
    ex_x4 = aut.to_expr(x4_over_int)
    ex_x5 = aut.to_expr(x5_over_int)

    ex_t1 = aut.to_expr(t1_over_int)
    ex_t2 = aut.to_expr(t2_over_int)
    ex_t3 = aut.to_expr(t3_over_int)
    ex_t4 = aut.to_expr(t4_over_int)
    ex_t5 = aut.to_expr(t5_over_int)


def dump_graph_as_figure(g):
    """Create a PDF file showing the graph `g`."""
    h, _ = sym_enum._format_nx(g)
    pd = nx.drawing.nx_pydot.to_pydot(h)
    pd.write_pdf('game_states.pdf')

if __name__ == '__main__':
    # testing winning set computation
    # define the specs here
    # ego_spec, test_spec = simple_test_specs()
    # system
    ex = 5 # Abstraction for the merge example
    if ex == 1:      # Simple FSM
        w_set = WinningSet()
        fsm = w_set.make_labeled_fsm()
        gr_spec = w_set.spec_from_fsm(fsm)
        aut = w_set.make_compatible_automaton(gr_spec)
        ctrl = synth.synthesize(gr_spec)
        assert ctrl is not None, 'unrealizable'
        specs.moore = True
        specs.qinit = r'\E \A'

    elif ex == 2:     # Simple specification
        w_set = WinningSet()
        ego_spec, test_spec = simple_test_specs()
        gr_spec = make_grspec(ego_spec, test_spec)
        w_set.set_spec(gr_spec)
        aut = w_set.make_compatible_automaton(gr_spec)

    elif ex == 3:
        w_set = WinningSet()
        gr_spec = example_win_set()
        # pdb.set_trace()
        w_set.set_spec(gr_spec)
        aut = w_set.make_compatible_automaton(gr_spec)
        # convert_bools(aut)

    elif ex == 4:
        w_set = WinningSet()
        aut = example_win_set3()

    elif ex==5: # Constructing abstraction for the merge example
        ego_spec, test_spec = spec_merge_in_front()#test_spec()#specs_for_entire_track(5)
        gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
        # invert the tester and the system
        print(gr_spec.pretty())
        # print(test_spec.pretty())
        w_set = WinningSet()
        w_set.set_spec(gr_spec)
        aut = w_set.make_compatible_automaton(gr_spec)

    winning_set = w_set.find_winning_set(aut)
    # ipdb.set_trace()
    # (x,y), (x1, y1), (x2,y2) are the positions of the system under test, the leading tester car, and the second tester car respectively. Domains of the position values can be found in the variable declarations in the specs_for_entire_track() function.
    state = {'x': 1, 'y': 1, 'x1': 1}#, 'y1':2, 'x2':1, 'y2': 2}  # To check if a state is in the winning set, pass all values in dictionary form. Each dictionary corresponds to one state.
    check_bdd = w_set.check_state_in_winset(aut, winning_set, state) # Check-bdd is a boolean. True implies that state is in the winning set.
    ipdb.set_trace()
    if check_bdd:
        print("State is in the winning set")
    else:
        print("State is not in the winning set")
    # shield_dict = w_set.synthesize_shield()
