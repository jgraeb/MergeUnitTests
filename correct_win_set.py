#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct  2 15:29:45 2021

@author: apurvabadithela
"""
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
import pickle
import os
from omega.games import enumeration as enum
from omega.symbolic import enumeration as sym_enum
import omega.logic.syntax as stx
from copy import deepcopy
import pdb
from omega.games import gr1
from omega.symbolic import fixpoint as fx
from omega.symbolic import fol as _fol
from omega.symbolic import prime as prm
from tulip.interfaces.omega import _grspec_to_automaton, _strategy_to_state_annotated
from tulip import spec
from tulip import transys
from tulip.synth import sys_to_spec
import logging
from tulip import transys, spec, synth
import tulip.interfaces.omega as omega_intf

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
    
    specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                    env_safe, sys_safe, env_prog, sys_prog)
    specs.qinit = r'\A \E'
    specs.moore = False
    specs.plus_one = False
    # Mealy / Moore specifications; While+1; system cannot violate it's spec
    return specs


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


# Specifications for automnaton
def specs_two_testers(tracklength):
    # st()
    sys_vars = {}
    sys_vars['x'] = (1, tracklength) # 0: system is much behind the second tester car, tracklength+1: system is much further than the first tester car
    sys_vars['y'] = (1,2)
    sys_init = {'x='+str(1), 'y='+str(1)}
    sys_prog = {'y=2'} # Eventually, the system should merge
    sys_safe = set()

    # Dynamics for merging into adjacent track:
    for ii in range(1,tracklength):
        sys_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=2))'}
        # sys_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2)|| (x='+str(ii+1)+' && y=1))'}
        sys_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2))'}

        # Pause system after merge
        # sys_safe |= {'(x='+str(ii)+' && y=2) -> X(x='+str(ii)+' && y=2)'}

    # sys_safe |= {'x=0 -> X(x=0 && x=1)'}
    # Dynamics of the system in the last cell:
    sys_safe |= {'(x='+str(tracklength)+' && y=1) -> X(x=' + str(tracklength) + ' && y = 1)'}
    sys_safe |= {'(x='+str(tracklength)+' && y=2) -> X(x=' + str(tracklength) + ' && y = 2)'}

    # testers
    tester_vars = {}
    tester_vars['x1'] = (2,tracklength)
    tester_vars['y1'] = (1,2)
    tester_vars['x2'] = (1,tracklength-1)
    tester_vars['y2'] = (1,2)
    tester_init = {'x1='+str(3), 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    tester_prog = set()
    merge_spec = "(x=2 && x1=3 && x2=2 && y=2 && y1=2 && y2=2)"
    # Freeze final specs:
    tester_safe = set()
    # tester_safe |= {merge_spec + " -> X(" + merge_spec+")"}
    # sys_safe |= {merge_spec + " -> X(" + merge_spec+")"}
    # sys_merge_spec = "(x=2 && y=2)"
    for ki in range(2,tracklength-1):
        new_merge_state = "(x="+str(ki+1)+ " && x1=" + str(ki+2) + " && x2=" + str(ki) + "&& y=2 && y1=2 && y2=2)"
        # tester_safe |= {new_merge_state + " -> X(" + new_merge_state+")"}
        # sys_safe |= {new_merge_state + " -> X(" + new_merge_state +")"}
        merge_spec += "|| " + new_merge_state
        # sys_merge_spec += ' || (y=2 && x= '+str(ki+1)+')'
    tester_prog |= {merge_spec}
    
    
    # tester_prog |= {sys_merge_spec}
    # tester_prog |= {'(y=2 && y1=2 && y2=2)'}

    # No collision with other vehicles:
    # for yi in range(1,3):
    #     for xi in range(1, tracklength+1):
    #         # Ignoring the end points:
    #         if xi!= tracklength:
    #             tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
    #             tester_safe |= {'!(x='+str(xi)+' && x2 ='+str(xi)+' && y= '+str(yi)+ ' && y2 = '+str(yi)+')'}
    #             sys_safe |= {'!(x='+str(xi)+' && x2 ='+str(xi)+' && y= '+str(yi)+ ' && y2 = '+str(yi)+')'}
    #         if xi != 1:
    #             tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
    #             tester_safe |= {'!(x='+str(xi)+' && x1 ='+str(xi)+' && y= '+str(yi)+ ' && y1 = '+str(yi)+')'}
    #             sys_safe |= {'!(x='+str(xi)+' && x1 ='+str(xi)+' && y= '+str(yi)+ ' && y1 = '+str(yi)+')'}
    #         # Endpoints:
    #         sys_safe |= {'!(x='+str(1)+' && x2 ='+str(1)+' && y= 2 && y2 = 2)'}
    #         sys_safe |= {'!(x='+str(1)+' && x2 ='+str(1)+' && y= 1 && y2 = 1)'}
    #         sys_safe |= {'!(x='+str(tracklength)+' && x1 ='+str(tracklength)+' && y= 2 && y1 = 2)'}
    #         sys_safe |= {'!(x='+str(tracklength)+' && x1 ='+str(tracklength)+' && y= 1 && y1 = 1)'}

    #         tester_safe |= {'!(x='+str(1)+' && x2 ='+str(1)+' && y= 2 && y2 = 2)'}
    #         tester_safe |= {'!(x='+str(tracklength)+' && x1 ='+str(tracklength)+' && y= 2 && y1 = 2)'}
    #         tester_safe |= {'!(x='+str(1)+' && x2 ='+str(1)+' && y= 1 && y2 = 1)'}
    #         tester_safe |= {'!(x='+str(tracklength)+' && x1 ='+str(tracklength)+' && y=1 && y1 = 1)'}
    
    tester_safe |= {'y1=2'} # testers stay in bottom lane
    tester_safe |= {'y2=2'}

    # Tester dynamics
    for ii in range(2,tracklength):
        tester_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1)|| (x1='+str(ii+1)+' && y1=2))'}
        tester_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2)|| (x1='+str(ii+1)+' && y1=1))'}
    
    for ii in range(1,tracklength):
        if ii != tracklength - 1:
            tester_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2)|| (x2='+str(ii+1)+' && y2=1))'}
            tester_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1)|| (x2='+str(ii+1)+' && y2=2))'}
            # print("skip")
        else:
            tester_safe |= {'(x2 =' + str(ii) + ' && y2 = 2) -> X(x2 = '+str(ii) +' && y2=2)'}
            tester_safe |= {'(x2 =' + str(ii) + ' && y2 = 1) -> X(x2 = '+str(ii) +' && y2=1)'}

    tester_safe |= {'(x1 =' + str(tracklength) + ' && y1 = 2) -> X(x1 = '+str(tracklength) +' && y1=2)'}
    tester_safe |= {'(x1 =' + str(tracklength) + ' && y1 = 1) -> X(x1 = '+str(tracklength) +' && y1=1)'}
    
    # T2 remains behind T1:
    # Doesn't really make much of a difference
    # tester_state = '(x1 = 2 && x2 = 1)'
    # for ii in range(3, tracklength+1):
    #     x2_states = 'x2=1'
    #     for x2 in range(2, ii):
    #         x2_states += '|| x2='+ str(x2)
    #     tester_state += '||( x1 = '+str(ii)+' && (' + x2_states + '))'
    # tester_safe |= {tester_state}
        
    # Terminal conditions: Once car merges, it remains merged.
    # for xi in range(1,tracklength+1):
    #     for x1i in range(2,tracklength):
    #         for x2i in range(1,x1i):
    #             other_st_i = '(x1='+str(x1i)+' && y1=2)&&(x2='+str(x2i)+' && y2=2)'
    #             sys_state_i = 'x = ' + str(xi)
    #             sys_safe |= {'(y=2 && '+ sys_state_i+')-> X('+ sys_state_i+' && y=2)'}
    #             for yi in range(1,3):
    #                 test_state_1i = '(x1 = '+ str(x1i) +' && y1 = ' + str(yi) + ')'
    #                 test_state_2i = '(x2 = '+ str(x2i) +' && y2 = ' + str(yi) + ')'
    #                 tester_safe |= {'((y=2 && x = '+str(xi)+') && '+str(test_state_1i)+') -> X('+test_state_1i+')'}
    #                 tester_safe |= {'((y=2 && x = '+str(xi)+') && '+str(test_state_2i)+') -> X('+test_state_2i+')'}
                    
    # # System reaches end of the road without merging:
    # sys_safe |= {'(y=1 && x=' + str(tracklength)+') -> X(y=1 && x=' + str(tracklength) +')'}
    # tester_safe |= {'((y=1 && x = '+str(tracklength)+') && '+str(test_state_1i)+') -> X('+test_state_1i+')'} # system reaches end of the road / no more merges
    # tester_safe |= {'((y=1 && x = '+str(tracklength)+') && '+str(test_state_2i)+') -> X('+test_state_2i+')'} # system reaches end of the road / no more merges
                    
    # Synthesize specs
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    # st()

    return ego_spec, test_spec

# System and environment have been switched:
def specs_for_entire_track(tracklength):
    # st()
    sys_vars = {}
    sys_vars['x'] = (1, tracklength) # 0: system is much behind the second tester car, tracklength+1: system is much further than the first tester car
    sys_vars['y'] = (1,2)
    sys_init = {'x='+str(1), 'y='+str(1)}
    sys_prog = {'y=2'} # Eventually, the system should merge
    sys_safe = set()

    # Dynamics for merging into adjacent track:
    for ii in range(1,tracklength-1):
        sys_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=2))'}
        sys_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2)|| (x='+str(ii+1)+' && y=1))'}
    # sys_safe |= {'x=0 -> X(x=0 && x=1)'}
    sys_safe |= {'x='+str(tracklength-1)+' -> X(x='+str(tracklength-1)+' && x='+str(tracklength)+')'}
    sys_safe |= {'x='+str(tracklength)+'-> X(x='+str(tracklength)+')'}

    # testers
    tester_vars = {}
    tester_vars['x1'] = (2,tracklength)
    tester_vars['y1'] = (1,2)
    tester_vars['x2'] = (1,tracklength-1)
    tester_vars['y2'] = (1,2)
    tester_init = {'x1='+str(2), 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    tester_prog = set()
    merge_spec = '((x=1) && (x1=3) && (x2=1) && (y=2 && y1=2 && y2=2))'
    for ki in range(1,tracklength-1):
        merge_spec = merge_spec + ' || ((x='+str(ki+1)+') && (x1='+str(ki+2)+') && (x2='+str(ki)+') && (y=2 && y1=2 && y2=2))'
    tester_prog |= {merge_spec}
    # tester_prog |= {'y=2'}
    # sys_prog |= {merge_spec}
    tester_safe = set()

    # No collision with other vehicles:
    for yi in range(1,3):
        for xi in range(1, tracklength+1):
            if xi!= tracklength:
                # tester_safe |= {'(x1='+str(xi)+ ' && y1= '+str(yi)+ ') -> X'}
                # sys_safe |= {'!(x='+str(xi)+' && x2 ='+str(xi)+' && y= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                # tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+ ') -> X(!(x2 ='+str(xi)+ ' && y2 = '+str(yi)+'))'}
                # sys_safe |= {'(x='+str(xi)+' && y= '+str(yi)+ ') -> X(!(x2 ='+str(xi)+ ' && y2 = '+str(yi)+'))'}

                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                tester_safe |= {'!(x='+str(xi)+' && x2 ='+str(xi)+' && y= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                sys_safe |= {'!(x='+str(xi)+' && x2 ='+str(xi)+' && y= '+str(yi)+ ' && y2 = '+str(yi)+')'}

            if xi != 1:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                # tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+ ') -> X(!(x1 ='+str(xi)+ ' && y1 = '+str(yi)+'))'}
                # sys_safe |= {'(x='+str(xi)+' && y= '+str(yi)+ ') -> X(!(x1 ='+str(xi)+ ' && y1 = '+str(yi)+'))'}
                tester_safe |= {'!(x='+str(xi)+' && x1 ='+str(xi)+' && y= '+str(yi)+ ' && y1 = '+str(yi)+')'}
                sys_safe |= {'!(x='+str(xi)+' && x1 ='+str(xi)+' && y= '+str(yi)+ ' && y1 = '+str(yi)+')'}
    
    sys_safe |= {'!(x='+str(1)+' && x2 ='+str(1)+' && y= 2 && y2 = 2)'}
    sys_safe |= {'!(x='+str(1)+' && x2 ='+str(1)+' && y= 1 && y2 = 1)'}
    sys_safe |= {'!(x='+str(tracklength)+' && x1 ='+str(tracklength)+' && y= 2 && y1 = 2)'}
    sys_safe |= {'!(x='+str(tracklength)+' && x1 ='+str(tracklength)+' && y= 1 && y1 = 1)'}

    tester_safe |= {'!(x='+str(1)+' && x2 ='+str(1)+' && y= 2 && y2 = 2)'}
    tester_safe |= {'!(x='+str(tracklength)+' && x1 ='+str(tracklength)+' && y= 2 && y1 = 2)'}
    tester_safe |= {'!(x='+str(1)+' && x2 ='+str(1)+' && y= 1 && y2 = 1)'}
    tester_safe |= {'!(x='+str(tracklength)+' && x1 ='+str(tracklength)+' && y=1 && y1 = 1)'}
    
    tester_safe |= {'!(y1=1) && !(y2=1)'} # testers stay in bottom lane

    # Tester dynamics
    for ii in range(2,tracklength-1):
        tester_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1)|| (x1='+str(ii+1)+' && y1=2))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2)|| (x2='+str(ii+1)+' && y2=1))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1)|| (x2='+str(ii+1)+' && y2=2))'}
        tester_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2)|| (x1='+str(ii+1)+' && y1=1))'}
    # tester_safe |= {'!(x1='+str(tracklength)+' && x2=0)'}
    tester_safe |= {'(x2='+str(tracklength)+' && y2=2) -> X(x2='+str(tracklength)+' && y2=2)'}
    tester_safe |= {'(x2='+str(tracklength)+' && y2=1) -> X(x2='+str(tracklength)+' && y2=1)'}    
    
    tester_safe |= {'(x2=1 && y2=2) -> X((x2=1 && y2=2) || (x2=2 && y2=2) || (x2=2 && y2=1))'}
    tester_safe |= {'(x2=1 && y2=1) -> X((x2=1 && y2=1) || (x2=2 && y2=2) || (x2=2 && y2=1))'}    
    
    tester_safe |= {'(x1='+str(tracklength)+' && y1=2) -> X(x1='+str(tracklength)+' && y1=2)'}
    tester_safe |= {'(x1='+str(tracklength)+' && y1=1) -> X(x1='+str(tracklength)+' && y1=1)'}

    tester_safe |= {'(x1='+str(tracklength-1)+' && y1=1) -> X((x1='+str(tracklength-1)+' && y1=1)|| (x1='+str(tracklength)+' && y1=2) || (x1='+str(tracklength)+' && y1=1)) '}
    tester_safe |= {'(x1='+str(tracklength-1)+' && y1=2) -> X((x1='+str(tracklength-1)+' && y1=2)|| (x1='+str(tracklength)+' && y1=1) || (x1='+str(tracklength)+' && y1=2)) '}
    #  tester_safe |= {'(x2=0) -> X(x2=0 || x2=1)'}

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


# Winning set for where the car merges if it gets a chance:
def specs_car_merge(tracklength):
    sys_vars = {}
    sys_vars['x'] = (1, tracklength) # 0: system is much behind the second tester car, tracklength+1: system is much further than the first tester car
    sys_vars['y'] = (1,2)
    sys_init = {'x='+str(1), 'y='+str(1)}
    sys_prog = set()
    sys_prog |= {'y=2'} # Eventually, the system should merge
    sys_safe = set()

    # Dynamics for merging into adjacent track:
    for ii in range(1,tracklength):
        sys_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=2))'}
        sys_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2)|| (x='+str(ii+1)+' && y=1))'}
    # sys_safe |= {'x=0 -> X(x=0 && x=1)'}
    # sys_safe |= {'(x='+str(tracklength-1)+' && y=1) -> X((x='+str(tracklength-1)+' && y=1)|| (x='+str(tracklength)+' && y=1))'}
    # sys_safe |= {'(x='+str(tracklength-1)+' && y=2) -> X((x='+str(tracklength-1)+' && y=2) || (x='+str(tracklength)+' && y=2))'}
    sys_safe |= {'(x='+str(tracklength)+' && y=2 )-> X(x='+str(tracklength)+' && y=2)'}
    sys_safe |= {'(x='+str(tracklength)+' && y=1 )-> X(x='+str(tracklength)+' && y=1)'}

    # If there is no env car in the kitty-corner lane, the car merges:
    for xi in range(1, tracklength-1):
        merge_if_possible = '(!(y1=2 && x1 = '+str(xi+1) +') && !(y2=2 && x2 = '+str(xi+1) +')) -> X(y=2 && x='+str(xi+1)+')'
        sys_safe |= {merge_if_possible}
        
    # Last cell merge:
    merge_if_possible = '(!(y1=2 && x1 = '+str(tracklength) +')) -> X(y=2 && x='+str(tracklength)+')'
    sys_safe |= {merge_if_possible}
    
    # Don't start until x1 is in front:
    sys_safe |= {'(x1=2 && y1=2) -> X(x=1 && y=1)'}
    
    # S - -
    # T T - 
    # Never reach the end state:
    # sys_safe |= {'!(x=3 && y=1)'}
    
    # testers
    tester_vars = {}
    tester_vars['x1'] = (2,tracklength)
    tester_vars['y1'] = (1,2)
    tester_vars['x2'] = (1,tracklength-1)
    tester_vars['y2'] = (1,2)
    tester_init = {'x1='+str(2), 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    tester_prog = set()
    tester_safe = set()

    # - - -
    # T S T
    # 
    #   1 2 3
    # 1 - - -
    # 2 - - -
    #
    # 1) 
    # - - -
    # T S T
    
    # 2) 
    # S - -
    # T _ T
    
    # 3) 
    # - - S -
    # - T _ T
    
    # [](x=2 /\ y=2 -> x1=3 /\ y=2 /\ x2=1  /\ y2 =2)
    # merge_spec = '((x=2 && y=2 && x1=3 && x2=1 && y1=2 && y2=2))'
    merge_spec = '((x=2 && y=2) -> (x1=3 && x2=1 && y1=2 && y2=2))'
    for ki in range(2,tracklength-1):
        merge_spec = merge_spec + ' || ((x='+str(ki+1)+' && y=2) -> ((x1='+str(ki+2)+' && (x2='+str(ki)+') && y1=2 && y2=2)))'
    tester_safe |= {merge_spec}
    
    # T2 shouldn't reach the last cell unless T1 merges:
    tester_safe |= {'(y=1 -> X !(x2=2))'}
    tester_prog |= {'(x1=3 && x2=1 && y1=2 && y2=2)'}
    # tester_prog |= {'(x=2 && y=2)'}
    # tester_prog |= {}
    # <>[](!(x=2 && y=2)) \/ []<>(x1=3 && x2=1 && y1=2 && y2=2)
    
    # No collision with other vehicles:
    for yi in range(1,3):
        for xi in range(1, tracklength+1):
            if xi!= tracklength:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+') -> X(!(x2 ='+str(xi)+'&& y2 = '+str(yi)+'))'}
                sys_safe |= {'(x2 ='+str(xi)+' && y2 = '+str(yi)+') -> X(!(x='+str(xi)+' && y= '+str(yi)+ '))'}

            if xi != 1:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                # tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+ ') -> X(!(x1 ='+str(xi)+ ' && y1 = '+str(yi)+'))'}
                # sys_safe |= {'(x='+str(xi)+' && y= '+str(yi)+ ') -> X(!(x1 ='+str(xi)+ ' && y1 = '+str(yi)+'))'}
                tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+') -> X(!(x1 ='+str(xi)+'&& y1 = '+str(yi)+'))'}
                sys_safe |= {'(x1 ='+str(xi)+' && y1 = '+str(yi)+') -> X(!(x='+str(xi)+' && y = '+str(yi)+ '))'}
    
    sys_safe |= {'(x2 ='+str(1)+'&& y2 = 2) -> X(!(x='+str(1)+' && y = 2))'}
    sys_safe |= {'(x2 ='+str(1)+'&& y2 = 1) -> X(!(x='+str(1)+' && y = 1))'}
    sys_safe |= {'(x1 ='+str(tracklength)+'&& y1 = 2) -> X(!(x='+str(tracklength)+' && y= 2))'}
    sys_safe |= {'(x1 ='+str(tracklength)+'&& y1 = 1) -> X(!(x='+str(tracklength)+' && y= 1))'}

    tester_safe |= {'(x ='+str(1)+'&& y = 2) -> X(!(x2='+str(1)+' && y2 = 2))'}
    tester_safe |= {'(x ='+str(1)+'&& y = 1) -> X(!(x2='+str(1)+' && y2 = 1))'}
    tester_safe |= {'(x ='+str(tracklength)+'&& y = 2) -> X(!(x1 ='+str(tracklength)+' && y1 = 2))'}
    tester_safe |= {'(x ='+str(tracklength)+' && y = 1) -> X(!(x1 ='+str(tracklength)+' && y1 = 1))'}
    
    tester_safe |= {'!(y1=1) && !(y2=1)'} # testers stay in bottom lane

    # Tester dynamics
    for ii in range(2,tracklength-1):
        tester_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1)|| (x1='+str(ii+1)+' && y1=2))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2)|| (x2='+str(ii+1)+' && y2=1))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1)|| (x2='+str(ii+1)+' && y2=2))'}
        tester_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2)|| (x1='+str(ii+1)+' && y1=1))'}
    # tester_safe |= {'!(x1='+str(tracklength)+' && x2=0)'}
    tester_safe |= {'(x2='+str(tracklength)+' && y2=2) -> X(x2='+str(tracklength)+' && y2=2)'}
    tester_safe |= {'(x2='+str(tracklength)+' && y2=1) -> X(x2='+str(tracklength)+' && y2=1)'}    
    
    tester_safe |= {'(x2=1 && y2=2) -> X((x2=1 && y2=2) || (x2=2 && y2=2) || (x2=2 && y2=1))'}
    tester_safe |= {'(x2=1 && y2=1) -> X((x2=1 && y2=1) || (x2=2 && y2=2) || (x2=2 && y2=1))'}    
    
    tester_safe |= {'(x1='+str(tracklength)+' && y1=2) -> X(x1='+str(tracklength)+' && y1=2)'}
    tester_safe |= {'(x1='+str(tracklength)+' && y1=1) -> X(x1='+str(tracklength)+' && y1=1)'}

    tester_safe |= {'(x1='+str(tracklength-1)+' && y1=1) -> X((x1='+str(tracklength-1)+' && y1=1)|| (x1='+str(tracklength)+' && y1=2) || (x1='+str(tracklength)+' && y1=1)) '}
    tester_safe |= {'(x1='+str(tracklength-1)+' && y1=2) -> X((x1='+str(tracklength-1)+' && y1=2)|| (x1='+str(tracklength)+' && y1=1) || (x1='+str(tracklength)+' && y1=2)) '}
    #  tester_safe |= {'(x2=0) -> X(x2=0 || x2=1)'}

    # Terminal conditions: Once car merges, it remains merged.
    for xi in range(1,tracklength):
        for x1i in range(1,tracklength):
            for x2i in range(0,x1i):
                other_st_i = '(x = '+str(xi)+') && (x1='+str(x1i)+')&&(x2='+str(x2i)+')'
                sys_safe |= {'((y=2 && y1=2 && y2=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2 && y1=2 && y2=2))'}
                tester_safe |= {'((y=2 && y1=2 && y2=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2 && y1=2 && y2=2))'}

    # Synthesize specs
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    return ego_spec, test_spec

# Car merges in front:
def specs_car_merge_front(tracklength):
    sys_vars = {}
    sys_vars['x'] = (1, tracklength) # 0: system is much behind the second tester car, tracklength+1: system is much further than the first tester car
    sys_vars['y'] = (1,2)
    sys_init = {'x='+str(1), 'y='+str(1)}
    sys_prog = set()
    sys_prog |= {'y=2'} # Eventually, the system should merge
    sys_safe = set()

    # Dynamics for merging into adjacent track:
    for ii in range(1,tracklength):
        sys_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=2))'}
        sys_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2)|| (x='+str(ii+1)+' && y=1))'}
    
    sys_safe |= {'(x='+str(tracklength)+' && y=2 )-> X(x='+str(tracklength)+' && y=2)'}
    sys_safe |= {'(x='+str(tracklength)+' && y=1 )-> X(x='+str(tracklength)+' && y=1)'}

    # If there is no env car in the kitty-corner lane, the car merges:
    for xi in range(1, tracklength-1):
        merge_if_possible = '(!(y1=2 && x1 = '+str(xi+1) +') && !(y2=2 && x2 = '+str(xi+1) +')) -> X(y=2 && x='+str(xi+1)+')'
        sys_safe |= {merge_if_possible}
        
    # Last cell merge:
    merge_if_possible = '(!(y1=2 && x1 = '+str(tracklength) +')) -> X(y=2 && x='+str(tracklength)+')'
    sys_safe |= {merge_if_possible}
    
    # Don't start until x1 is in front:
    # sys_safe |= {'(x1=2 && y1=2) -> X(x=1 && y=1)'}
    
    # testers
    tester_vars = {}
    tester_vars['x1'] = (2,tracklength)
    tester_vars['y1'] = (1,2)
    tester_vars['x2'] = (1,tracklength-1)
    tester_vars['y2'] = (1,2)
    tester_init = {'x1='+str(2), 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    tester_prog = set()
    tester_safe = set()

    # [](x=2 /\ y=2 -> x1=3 /\ y=2 /\ x2=1  /\ y2 =2)
    # merge_spec = '((x=2 && y=2 && x1=3 && x2=1 && y1=2 && y2=2))'
    merge_spec = '((x=2 && y=2) -> (x2=1 && y2=2))'
    for ki in range(2,tracklength):
        merge_spec = merge_spec + ' || ((x='+str(ki+1)+' && y=2) -> ((x1='+str(ki)+' && y1=2) || (x2='+str(ki)+' && y2=2)))'
    tester_safe |= {merge_spec}
    
    # T2 shouldn't reach the last cell unless T1 merges:
    # tester_safe |= {'(y=1 -> X !(x2=2))'}
    # tester_prog |= {'(x1=3 && x2=1 && y1=2 && y2=2)'}
    # No collision with other vehicles:
    for yi in range(1,3):
        for xi in range(1, tracklength+1):
            if xi!= tracklength:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+') -> X(!(x2 ='+str(xi)+'&& y2 = '+str(yi)+'))'}
                sys_safe |= {'(x2 ='+str(xi)+' && y2 = '+str(yi)+') -> X(!(x='+str(xi)+' && y= '+str(yi)+ '))'}

            if xi != 1:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                # tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+ ') -> X(!(x1 ='+str(xi)+ ' && y1 = '+str(yi)+'))'}
                # sys_safe |= {'(x='+str(xi)+' && y= '+str(yi)+ ') -> X(!(x1 ='+str(xi)+ ' && y1 = '+str(yi)+'))'}
                tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+') -> X(!(x1 ='+str(xi)+'&& y1 = '+str(yi)+'))'}
                sys_safe |= {'(x1 ='+str(xi)+' && y1 = '+str(yi)+') -> X(!(x='+str(xi)+' && y = '+str(yi)+ '))'}
    
    sys_safe |= {'(x2 ='+str(1)+'&& y2 = 2) -> X(!(x='+str(1)+' && y = 2))'}
    sys_safe |= {'(x2 ='+str(1)+'&& y2 = 1) -> X(!(x='+str(1)+' && y = 1))'}
    sys_safe |= {'(x1 ='+str(tracklength)+'&& y1 = 2) -> X(!(x='+str(tracklength)+' && y= 2))'}
    sys_safe |= {'(x1 ='+str(tracklength)+'&& y1 = 1) -> X(!(x='+str(tracklength)+' && y= 1))'}

    tester_safe |= {'(x ='+str(1)+'&& y = 2) -> X(!(x2='+str(1)+' && y2 = 2))'}
    tester_safe |= {'(x ='+str(1)+'&& y = 1) -> X(!(x2='+str(1)+' && y2 = 1))'}
    tester_safe |= {'(x ='+str(tracklength)+'&& y = 2) -> X(!(x1 ='+str(tracklength)+' && y1 = 2))'}
    tester_safe |= {'(x ='+str(tracklength)+' && y = 1) -> X(!(x1 ='+str(tracklength)+' && y1 = 1))'}
    
    tester_safe |= {'!(y1=1) && !(y2=1)'} # testers stay in bottom lane

    # Tester dynamics
    for ii in range(2,tracklength-1):
        tester_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1)|| (x1='+str(ii+1)+' && y1=2))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2)|| (x2='+str(ii+1)+' && y2=1))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1)|| (x2='+str(ii+1)+' && y2=2))'}
        tester_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2)|| (x1='+str(ii+1)+' && y1=1))'}
    # tester_safe |= {'!(x1='+str(tracklength)+' && x2=0)'}
    tester_safe |= {'(x2='+str(tracklength)+' && y2=2) -> X(x2='+str(tracklength)+' && y2=2)'}
    tester_safe |= {'(x2='+str(tracklength)+' && y2=1) -> X(x2='+str(tracklength)+' && y2=1)'}    
    
    tester_safe |= {'(x2=1 && y2=2) -> X((x2=1 && y2=2) || (x2=2 && y2=2) || (x2=2 && y2=1))'}
    tester_safe |= {'(x2=1 && y2=1) -> X((x2=1 && y2=1) || (x2=2 && y2=2) || (x2=2 && y2=1))'}    
    
    tester_safe |= {'(x1='+str(tracklength)+' && y1=2) -> X(x1='+str(tracklength)+' && y1=2)'}
    tester_safe |= {'(x1='+str(tracklength)+' && y1=1) -> X(x1='+str(tracklength)+' && y1=1)'}

    tester_safe |= {'(x1='+str(tracklength-1)+' && y1=1) -> X((x1='+str(tracklength-1)+' && y1=1)|| (x1='+str(tracklength)+' && y1=2) || (x1='+str(tracklength)+' && y1=1)) '}
    tester_safe |= {'(x1='+str(tracklength-1)+' && y1=2) -> X((x1='+str(tracklength-1)+' && y1=2)|| (x1='+str(tracklength)+' && y1=1) || (x1='+str(tracklength)+' && y1=2)) '}
    #  tester_safe |= {'(x2=0) -> X(x2=0 || x2=1)'}

    # Terminal conditions: Once car merges, it remains merged.
    # for xi in range(1,tracklength):
    #     for x1i in range(1,tracklength):
    #         for x2i in range(0,x1i):
    #             other_st_i = '(x = '+str(xi)+') && (x1='+str(x1i)+')&&(x2='+str(x2i)+')'
    #             sys_safe |= {'((y=2 && y1=2 && y2=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2 && y1=2 && y2=2))'}
    #             tester_safe |= {'((y=2 && y1=2 && y2=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2 && y1=2 && y2=2))'}

    # Synthesize specs
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    return ego_spec, test_spec

# Car merges in back:
def specs_car_merge_back(tracklength):
    sys_vars = {}
    sys_vars['x'] = (1, tracklength) # 0: system is much behind the second tester car, tracklength+1: system is much further than the first tester car
    sys_vars['y'] = (1,2)
    sys_init = {'x='+str(1), 'y='+str(1)}
    sys_prog = set()
    sys_prog |= {'y=2'} # Eventually, the system should merge
    sys_safe = set()

    # Dynamics for merging into adjacent track:
    for ii in range(1,tracklength):
        sys_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=2))'}
        sys_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2)|| (x='+str(ii+1)+' && y=1))'}
    sys_safe |= {'(x='+str(tracklength)+' && y=2 )-> X(x='+str(tracklength)+' && y=2)'}
    sys_safe |= {'(x='+str(tracklength)+' && y=1 )-> X(x='+str(tracklength)+' && y=1)'}

    # If there is no env car in the kitty-corner lane, the car merges:
    for xi in range(1, tracklength-1):
        merge_if_possible = '(!(y1=2 && x1 = '+str(xi+1) +') && !(y2=2 && x2 = '+str(xi+1) +')) -> X(y=2 && x='+str(xi+1)+')'
        sys_safe |= {merge_if_possible}
        
    # Last cell merge:
    merge_if_possible = '(!(y1=2 && x1 = '+str(tracklength) +')) -> X(y=2 && x='+str(tracklength)+')'
    sys_safe |= {merge_if_possible}
    
    # Don't start until x1 is in front:
    sys_safe |= {'(x1=2 && y1=2) -> X(x=1 && y=1)'}
    
    # testers
    tester_vars = {}
    tester_vars['x1'] = (2,tracklength)
    tester_vars['y1'] = (1,2)
    tester_vars['x2'] = (1,tracklength-1)
    tester_vars['y2'] = (1,2)
    tester_init = {'x1='+str(2), 'y1='+str(2), 'x2='+str(1), 'y2='+str(2)}
    tester_prog = set()
    tester_safe = set()
    
    # [](x=2 /\ y=2 -> x1=3 /\ y=2 /\ x2=1  /\ y2 =2)
    # merge_spec = '((x=2 && y=2 && x1=3 && x2=1 && y1=2 && y2=2))'
    merge_spec = ''
    for ki in range(2,tracklength):
        if ki == tracklength-1:
            if merge_spec == '':
                merge_spec = '(x='+str(ki)+' && y=2) -> ((x1='+str(ki+1)+' && y1=2))'
            else: 
                merge_spec = merge_spec + ' || ((x='+str(ki)+' && y=2) -> (x1='+str(ki+1)+' && y1=2))'
        else:
            if merge_spec == '':
                merge_spec = '(x='+str(ki)+' && y=2) -> ((x1='+str(ki+1)+' && y1=2) || (x2='+str(ki+1)+' && y2=2))'
            else: 
                merge_spec = merge_spec + '|| ((x='+str(ki)+' && y=2) -> ((x1='+str(ki+1)+' && y1=2) || (x2='+str(ki+1)+' && y2=2)))'
    tester_safe |= {merge_spec}
    
    # T2 shouldn't reach the last cell unless T1 merges:
    tester_safe |= {'(y=1 -> X !(x2=2))'}

    # No collision with other vehicles:
    for yi in range(1,3):
        for xi in range(1, tracklength+1):
            if xi!= tracklength:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+') -> X(!(x2 ='+str(xi)+'&& y2 = '+str(yi)+'))'}
                sys_safe |= {'(x2 ='+str(xi)+' && y2 = '+str(yi)+') -> X(!(x='+str(xi)+' && y= '+str(yi)+ '))'}

            if xi != 1:
                tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
                tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+') -> X(!(x1 ='+str(xi)+'&& y1 = '+str(yi)+'))'}
                sys_safe |= {'(x1 ='+str(xi)+' && y1 = '+str(yi)+') -> X(!(x='+str(xi)+' && y = '+str(yi)+ '))'}
    
    sys_safe |= {'(x2 ='+str(1)+'&& y2 = 2) -> X(!(x='+str(1)+' && y = 2))'}
    sys_safe |= {'(x2 ='+str(1)+'&& y2 = 1) -> X(!(x='+str(1)+' && y = 1))'}
    sys_safe |= {'(x1 ='+str(tracklength)+'&& y1 = 2) -> X(!(x='+str(tracklength)+' && y= 2))'}
    sys_safe |= {'(x1 ='+str(tracklength)+'&& y1 = 1) -> X(!(x='+str(tracklength)+' && y= 1))'}

    tester_safe |= {'(x ='+str(1)+'&& y = 2) -> X(!(x2='+str(1)+' && y2 = 2))'}
    tester_safe |= {'(x ='+str(1)+'&& y = 1) -> X(!(x2='+str(1)+' && y2 = 1))'}
    tester_safe |= {'(x ='+str(tracklength)+'&& y = 2) -> X(!(x1 ='+str(tracklength)+' && y1 = 2))'}
    tester_safe |= {'(x ='+str(tracklength)+' && y = 1) -> X(!(x1 ='+str(tracklength)+' && y1 = 1))'}
    
    tester_safe |= {'!(y1=1) && !(y2=1)'} # testers stay in bottom lane

    # Tester dynamics
    for ii in range(2,tracklength-1):
        tester_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1)|| (x1='+str(ii+1)+' && y1=2))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2)|| (x2='+str(ii+1)+' && y2=1))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1)|| (x2='+str(ii+1)+' && y2=2))'}
        tester_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2)|| (x1='+str(ii+1)+' && y1=1))'}
    # tester_safe |= {'!(x1='+str(tracklength)+' && x2=0)'}
    tester_safe |= {'(x2='+str(tracklength)+' && y2=2) -> X(x2='+str(tracklength)+' && y2=2)'}
    tester_safe |= {'(x2='+str(tracklength)+' && y2=1) -> X(x2='+str(tracklength)+' && y2=1)'}    
    
    tester_safe |= {'(x2=1 && y2=2) -> X((x2=1 && y2=2) || (x2=2 && y2=2) || (x2=2 && y2=1))'}
    tester_safe |= {'(x2=1 && y2=1) -> X((x2=1 && y2=1) || (x2=2 && y2=2) || (x2=2 && y2=1))'}    
    
    tester_safe |= {'(x1='+str(tracklength)+' && y1=2) -> X(x1='+str(tracklength)+' && y1=2)'}
    tester_safe |= {'(x1='+str(tracklength)+' && y1=1) -> X(x1='+str(tracklength)+' && y1=1)'}

    tester_safe |= {'(x1='+str(tracklength-1)+' && y1=1) -> X((x1='+str(tracklength-1)+' && y1=1)|| (x1='+str(tracklength)+' && y1=2) || (x1='+str(tracklength)+' && y1=1)) '}
    tester_safe |= {'(x1='+str(tracklength-1)+' && y1=2) -> X((x1='+str(tracklength-1)+' && y1=2)|| (x1='+str(tracklength)+' && y1=1) || (x1='+str(tracklength)+' && y1=2)) '}

    # Terminal conditions: Once car merges, it remains merged.
    # for xi in range(1,tracklength):
    #     for x1i in range(1,tracklength):
    #         for x2i in range(0,x1i):
    #             other_st_i = '(x = '+str(xi)+') && (x1='+str(x1i)+')&&(x2='+str(x2i)+')'
    #             sys_safe |= {'((y=2 && y1=2 && y2=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2 && y1=2 && y2=2))'}
    #             tester_safe |= {'((y=2 && y1=2 && y2=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2 && y1=2 && y2=2))'}

    # Synthesize specs
    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    return ego_spec, test_spec

# Check safety and progress assumptions:
# Check s |= A && G, and only put those states that are in the winning set:
def check_st_A_int_G(state, tracklength, mode):
    x = state['x']
    y = state['y'] 
    x1 = state['x1']
    y1 = state['y1'] 
    x2 = state['x2']
    y2 = state['y2']
    flg = False
    if mode == "between":
        if x1 == tracklength:
            spec = lambda x, y, x1, y1, x2, y2: ((x2==x or x2==x+1) and (x1==x2+2) and (y==1) and (y1==2) and (y2==2))
            flg = spec(x,y,x1,y1,x2,y2) 
        else:
            spec1 = lambda x, y, x1, y1, x2, y2: ((x2==x+1) and (y==1) and (y2==2)) or ((x1==x+1) and (y==1) and (y1==2))
            spec2 = lambda x, y, x1, y1, x2, y2: ((x2==x) and (x1==x2+2) and (y==1) and (y1==2) and (y2==2))
            flg = spec1(x,y,x1,y1,x2,y2) or spec2(x,y,x1,y1,x2,y2)
        spec_merged = lambda x, y, x1, y1, x2, y2: ((x2==x-1) and (x1==x2+2) and (y==2) and (y1==2) and (y2==2))
        flg = flg or spec_merged(x,y,x1,y1, x2, y2)
    
    elif mode == "front":
        if x == tracklength-2:
            spec1 = lambda x, y, x1, y1, x2, y2: (y==1 and y2==2 and x2==x and y1==2 and (x1==x+1 or x1==x+2))
            spec2 = lambda x, y, x1, y1, x2, y2: (y==1 and y2==2 and x2==x-1 and y1==2 and x1==x+1)
            flg = spec1(x,y,x1,y1,x2,y2) or spec2(x,y,x1,y1,x2,y2)
        elif x == tracklength-1:
            spec = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and x1==x)
            flg = spec(x,y,x1,y1,x2,y2)
        elif x < tracklength-2:
            spec = lambda x, y, x1, y1, x2, y2: (y2==2 and y==1 and (x2==x or x2==x+1)) or (y1==2 and y==1 and (x1==x or x1==x+1))
            flg = spec(x,y,x1,y1,x2,y2)
        spec_merged = lambda x, y, x1, y1, x2, y2: ((x2==x-1 or x1==x-1) and (y==2) and (y1==2) and (y2==2))
        flg = flg or spec_merged(x,y,x1,y1, x2, y2)
        
    elif mode == "back":
        if x == tracklength-2:
            spec = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and (x1==x+1 or x1==x+2) and (x2 != x+1))
            flg = spec(x,y,x1,y1,x2,y2)
        elif x < tracklength-2:
            spec1 = lambda x, y, x1, y1, x2, y2: ((y==1) and (y2==2) and ((x2==x+1) or (x2==x+2)))
            spec2 = lambda x, y, x1, y1, x2, y2: ((y==1) and (y1==2) and ((x1==x+1) or (x1==x+2 and x2!=x+1)))
            flg = spec1(x,y,x1,y1,x2,y2) or spec2(x,y,x1,y1,x2,y2)
        spec_merged = lambda x, y, x1, y1, x2, y2: ((x2==x+1 or x1==x+1) and (y==2) and (y1==2) and (y2==2))
        flg = flg or spec_merged(x,y,x1,y1, x2, y2)
    else:
        print("Enter the correct mode (between/front/back)")
    return flg

# Construct system action states that are in the tester winning set:
def construct_sys_win_states(tracklength, states_in_winset):
    sys_states_in_winset = []
    return sys_states_in_winset

def forward_step(tracklength, states_in_winset, x, y, x1, y1, x2, y2):
    sys_succ_in_winset = []
    tester_succ = forward_step_tester(tracklength, x, y, x1. y1, x2, y2)
    for ti in tester_succ:
        system_succ = forward_step_sys(tracklength, x, y, x1, y1, x2, y2)
        flg = True
        for si in system_succ:
            state = {'x': si[0], 'y': si[1], 'x1': ti[0], 'y1': ti[1], 'x2': ti[2], 'x3': ti[3]}
            if state not in states_in_winset:
                flg = False
                break
        if flg:
            sys_state = {'x': x, 'y': y, 'x1': x1, 'y1': y1, 'x2': x2, 'x3': y2}
            sys_succ_in_winset.append(sys_state)
    return sys_succ_in_winset

def forward_step_sys(tracklength, x, y, x1, y1, x2, y2):
    if x == tracklength:
        return [(x,y)]
    new_steps = [(x,y), (x+1, 1), (x+1, 2)]
    tester_states = [(x1,y1), (x2,y2)]
    for ti in tester_states:
        if ti in new_steps:
            new_steps.remove(ti)
    return new_steps

def forward_step_tester(tracklength, x, y, x1, y1, x2, y2):
    new_steps = [(x1,y1,x2,y2)]
    if x1 < tracklength:
        new_steps.append((x1+1, 1, x2, y2))
        new_steps.append((x1+1, 2, x2, y2))
    if x2 < tracklength-1:
        new_steps.append((x1, y1, x2+1, 1))
        new_steps.append((x1, y1, x2+1, 2))
    if x1 < tracklength and x2 < tracklength-1:
        new_steps.append((x1+1, 1, x2+1, 1))
        new_steps.append((x1+1, 2, x2+1, 2))
        new_steps.append((x1+1, 2, x2+1, 1))
        new_steps.append((x1+1, 1, x2+1, 2))
    
    for ti in new_steps:
        if ti[0] == x and ti[1] == y:
            new_steps.remove(ti)
            continue
        if ti[2] == x and ti[3] == y:
            new_steps.remove(ti)
            continue
    return new_steps

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

def dump_graph_as_figure(g):
    """Create a PDF file showing the graph `g`."""
    h, _ = sym_enum._format_nx(g)
    pd = nx.drawing.nx_pydot.to_pydot(h)
    pd.write_pdf('game_states.pdf')

def check_all_states_in_fp(tracklength, agentlist, w_set, winning_set, aut):
    # winning_set = w_set.find_winning_set(aut)
    print_flg = True
    num_test_agents = len(agentlist)
    states_in_winset = []
    states_outside_winset = []
    if num_test_agents == 1:
        for x in range(1,tracklength+1):
            for y in range(1,2+1):
                for x1 in range(1,tracklength+1):
                    state = {'x': x, 'y': y, 'x1': x1}
                    check_bdd = w_set.check_state_in_fp(aut, winning_set, state)
                    print(state)
                    print(check_bdd)
    # x2 < x1, since x2 is a second tester
    elif num_test_agents ==2:
        for x in range(1,tracklength+1):
            for y in range(1,2+1):
                for x1 in range(1,tracklength+1):
                    for x2 in range(1, x1):
                        state = {'x': x, 'y': y, agentlist[0]: x1, 'y1':2, agentlist[1]: x2, 'y2':2}
                        check_bdd = w_set.check_state_in_fp(aut, winning_set, state)
                        if check_bdd:
                            states_in_winset.append(state)
                        else:
                            states_outside_winset.append(state)
                        if print_flg: 
                            print(state)
                            print(check_bdd)
    else:
        print('Too many agents')
    return states_in_winset, states_outside_winset

# Filtering states in the winning set:
def check_all_states_in_winset(tracklength, agentlist, w_set, winning_set, aut, mode):
    # winning_set = w_set.find_winning_set(aut)
    num_test_agents = len(agentlist)
    states_in_winset = []
    states_outside_winset = []
    if num_test_agents == 1:
        for x in range(1,tracklength+1):
            for y in range(1,2+1):
                for x1 in range(1,tracklength+1):
                    state = {'x': x, 'y': y, 'x1': x1}
                    check_bdd = w_set.check_state_in_fp(aut, winning_set, state)
                    print(state)
                    print(check_bdd)
    # x2 < x1, since x2 is a second tester
    elif num_test_agents ==2:
        for x in range(1,tracklength+1):
            for y in range(1,2+1):
                for x1 in range(1,tracklength+1):
                    for x2 in range(1, x1):
                        state = {'x': x, 'y': y, agentlist[0]: x1, 'y1':2, agentlist[1]: x2, 'y2':2}
                        check_bdd = w_set.check_state_in_fp(aut, winning_set, state)
                        if check_bdd:
                            check_flg = check_st_A_int_G(state, tracklength, mode)
                            if check_flg:
                                states_in_winset.append(state)
                                print(state)
                                print(check_bdd)
                            else:
                                states_outside_winset.append(state)
                        else:
                            states_outside_winset.append(state)
    else:
        print('Too many agents')
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


def get_winset(tracklength, merge_setting):
    if merge_setting == "between":
        ego_spec, test_spec = specs_car_merge(tracklength) #spec_merge_in_front()#all_system(3)#spec_merge_in_front()#test_spec()#specs_for_entire_track(5)
        gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
        # print(gr_spec.pretty())
        w_set = WinningSet()
        w_set.set_spec(gr_spec)
        
    elif merge_setting == "front":
        ego_spec, test_spec = specs_car_merge_front(tracklength) #spec_merge_in_front()#all_system(3)#spec_merge_in_front()#test_spec()#specs_for_entire_track(5)
        gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
        # print(gr_spec.pretty())
        w_set = WinningSet()
        w_set.set_spec(gr_spec)

    elif merge_setting == "back":
        ego_spec, test_spec = specs_car_merge_back(tracklength) #spec_merge_in_front()#all_system(3)#spec_merge_in_front()#test_spec()#specs_for_entire_track(5)
        gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
        # print(gr_spec.pretty())
        w_set = WinningSet()
        w_set.set_spec(gr_spec)
   
    else:
        print("Incorrect merge setting (between/front/back) ")
    
    aut = w_set.make_compatible_automaton(gr_spec)
    # g = synthesize_some_controller(aut) 
    agentlist = ['x1', 'x2']
    fp = w_set.find_winning_set(aut)
    # print("Printing states in fixpoint: ")
    states_in_fp, states_out_fp = check_all_states_in_fp(tracklength, agentlist, w_set, fp, aut)
    print(" ")
    print("Printing states in winning set: ")
    states_in_winset, states_out_winset = check_all_states_in_winset(tracklength, agentlist, w_set, fp, aut, merge_setting)
    return states_in_winset

if __name__ == '__main__':
    ex = 4 # Abstraction for the merge example
    if ex==6: # Constructing abstraction for the merge example
        tracklength = 3
        ego_spec, test_spec = specs_two_testers(tracklength) #spec_merge_in_front()#all_system(3)#spec_merge_in_front()#test_spec()#specs_for_entire_track(5)
        gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
        print(gr_spec.pretty())
        w_set = WinningSet()
        w_set.set_spec(gr_spec)
        aut = w_set.make_compatible_automaton(gr_spec)
        # g = synthesize_some_controller(aut) 
        # if g is None:
        #     print("Inconsistent specifications")
        # else:
        #     print("Controller found")
        agentlist = ['x1', 'x2']
        pdb.set_trace()
        states_in_winset, states_out_winset = check_all_states(tracklength, agentlist, w_set, aut)
    
    if ex==5: # Constructing abstraction for the merge example
        tracklength = 10
        ego_spec, test_spec = specs_for_entire_track(tracklength) #spec_merge_in_front()#all_system(3)#spec_merge_in_front()#test_spec()#specs_for_entire_track(5)
        gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
        print(gr_spec.pretty())
        w_set = WinningSet()
        w_set.set_spec(gr_spec)
        aut = w_set.make_compatible_automaton(gr_spec)
        # g = synthesize_some_controller(aut) 
        agentlist = ['x1', 'x2']
        states_in_winset, states_out_winset = check_all_states(tracklength, agentlist, w_set, aut)
        
    if ex==4: # Constru
        tracklength = 4
        ego_spec, test_spec = specs_car_merge(tracklength) #spec_merge_in_front()#all_system(3)#spec_merge_in_front()#test_spec()#specs_for_entire_track(5)
        gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
        print(gr_spec.pretty())
        w_set = WinningSet()
        w_set.set_spec(gr_spec)

        
        aut = w_set.make_compatible_automaton(gr_spec)
        # g = synthesize_some_controller(aut) 
        agentlist = ['x1', 'x2']
        fp = w_set.find_winning_set(aut)
        print("Printing states in fixpoint: ")
        states_in_fp, states_out_fp = check_all_states_in_fp(tracklength, agentlist, w_set, fp, aut)
        print(" ")
        print("Printing states in winning set: ")
        mode="between"
        states_in_winset = check_all_states_in_winset(tracklength, agentlist, w_set, fp, aut, mode)
    # Check paranthesizing: 
    # ego_spec_init = r' /\ '.join(f'({e})' for e in ego_spec.init)
    
    pdb.set_trace()
    ego_spec_init = stx.conj(ego_spec.init)
    test_spec_init = stx.conj(test_spec.init)
    expr = rf'{ego_spec_init} /\ {winning_set} /\ {test_spec_init}'
    print(expr)
    winning_set_from_init = aut.add_expr(expr)
    sin, sout = check_all_states(tracklength, agentlist, winning_set_from_init, aut)
    
    pdb.set_trace()
    if omega_intf.is_circular(gr_spec):
        raise AssertionError('detected circularity in the specification')
    state = {'x': 1, 'y': 1, 'x1': 3, 'y1':2, 'x2':2, 'y2': 2}  # To check if a state is in the winning set, pass all values in dictionary form. Each dictionary corresponds to one state.
    check_bdd = w_set.check_state_in_fp(aut, winning_set, state) # Check-bdd is a boolean. True implies that state is in the winning set.

    if check_bdd:
        print("State is in the winning set")
    else:
        print("State is not in the winning set")
