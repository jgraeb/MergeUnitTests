#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Specifications for Intersection Example.

@author: Josefine Graebener
Oct 26 2021
"""
import sys
sys.path.append('..')
import numpy as np
import pdb
import logging
from tulip import transys, spec, synth
from tulip.interfaces.omega import _grspec_to_automaton
import tulip.interfaces.omega as omega_intf
from intersection import make_state_dictionary_for_specification

class Spec:
    def __init__(self,sys_vars,init,safety,progress):
        self.variables = sys_vars
        self.init = init
        self.safety = safety
        self.prog = progress

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

def synthesize_controller(spec):
    aut = _grspec_to_automaton(spec) #`temporal.Automaton` - compiled game with <>[] \/ []<> winning
    z, yij, xijk = gr1.solve_streett_game(aut)
    gr1.make_streett_transducer(z, yij, xijk, aut)
    ctrl = enum.action_to_steps(aut, env='env', sys='impl', qinit=aut.qinit)
    assert ctrl is not None, 'unrealizable'

def get_constants_from_map():
    # to do
    pass

def intersection_specs(state_dict):
    x_min_grid = 0
    x_max_grid = 7
    y_min_grid = 0
    y_max_grid = 7
    # bounds = [x_min_grid,x_max_grid, y_min_grid,y_max_grid]
    x_init = 7
    y_init = 4
    x_goal = 3
    y_goal = 0
    sys_vars = {}
    sys_vars['x'] = (x_min_grid, x_max_grid)
    sys_vars['y'] = (y_min_grid, y_max_grid)
    sys_init = {'x='+str(x_init)+' && y='+str(y_init)}
    sys_prog = set()
    sys_prog |= {'x='+str(x_goal)+' && y='+str(y_goal)}
    sys_safe = set()
    # add the dynamics for the system
    sys_safe |= add_dynamics(state_dict,[('x','y')], x_min_grid,x_max_grid, y_min_grid,y_max_grid)

    # testers
    x1_init = 0
    y2_init = 3
    x2_init = 3
    y1_init = 7
    tester_vars = {}
    tester_vars['x1'] = (x_min_grid, x_max_grid)
    tester_vars['y1'] = (y_min_grid, y_max_grid)
    tester_vars['x2'] = (x_min_grid, x_max_grid)
    tester_vars['y2'] = (y_min_grid, y_max_grid)
    tester_init = {'x1='+str(x1_init), 'y1='+str(y1_init), 'x2='+str(x2_init), 'y2='+str(y2_init)}
    tester_prog = set()
    tester_safe = set()
    # Add the dynamics
    tester_safe |= add_dynamics(state_dict, [('x1','y1'),('x2','y2')], x_min_grid,x_max_grid, y_min_grid,y_max_grid)
    # Add no collissions between any agents
    no_collisions_tester, no_collisions_sys = no_collisions(x_min_grid, x_max_grid, y_min_grid, y_max_grid)
    tester_safe |= no_collisions_tester
    sys_safe |= no_collisions_sys

    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    return ego_spec, test_spec

def no_collisions(x_min_grid, x_max_grid, y_min_grid, y_max_grid):
    '''
    Create the specifications to ensure no collisions.
    '''
    tester_safe = set()
    sys_safe = set()
    # No collision with other vehicles:
    for yi in range(y_min_grid,y_max_grid):
        for xi in range(x_min_grid, x_max_grid):
            tester_safe |= {'!(x1='+str(xi)+' && x2 ='+str(xi)+' && y1= '+str(yi)+ ' && y2 = '+str(yi)+')'}
            tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+') -> X(!(x1 ='+str(xi)+'&& y1 = '+str(yi)+'))'}
            tester_safe |= {'(x='+str(xi)+' && y= '+str(yi)+') -> X(!(x2 ='+str(xi)+'&& y2 = '+str(yi)+'))'}
            sys_safe |= {'(x2 ='+str(xi)+' && y2 = '+str(yi)+') -> X(!(x='+str(xi)+' && y= '+str(yi)+ '))'}
            sys_safe |= {'(x1 ='+str(xi)+' && y1 = '+str(yi)+') -> X(!(x='+str(xi)+' && y = '+str(yi)+ '))'}
    return tester_safe, sys_safe

def add_dynamics(state_dict, agent_var_list, x_min_grid,x_max_grid, y_min_grid,y_max_grid):
    '''
    Add the dynamics from the intersection grid to the specifications.
    '''
    dynamics_spec = set()
    for agent_var in agent_var_list:
        for ii in range(x_min_grid,x_max_grid):
            for jj in range(y_min_grid,y_max_grid):
                if (ii,jj) in state_dict:
                    next_steps_string = ''
                    for item in state_dict[(ii,jj)]:
                        if next_steps_string == '':
                            next_steps_string = next_steps_string + '('+str(agent_var[0])+'='+str(item[0])+' && '+str(agent_var[1])+' ='+str(item[0])+')'
                        else:
                            next_steps_string = next_steps_string + ' || ('+str(agent_var[0])+'='+str(item[0])+' && '+str(agent_var[1])+'='+str(item[0])+')'
                    dynamics_spec |= {'('+str(agent_var[0])+'='+str(ii)+' && '+str(agent_var[1])+'='+str(jj)+') -> X(('+ next_steps_string +'))'}

        # Make sure everything off map stays off map - NOT SURE ABOUT THIS PART
        # dynamics_spec |= {'('+str(agent_var[0])+'>'+str(x_max_grid)+')-> X('+str(agent_var[0])+'>'+str(x_max_grid)}
        # dynamics_spec |= {'('+str(agent_var[1])+'>'+str(y_max_grid)+')-> X('+str(agent_var[0])+'>'+str(y_max_grid)}
        # dynamics_spec |= {'('+str(agent_var[0])+'<'+str(x_min_grid)+')-> X('+str(agent_var[0])+'<'+str(x_min_grid)}
        # dynamics_spec |= {'('+str(agent_var[1])+'<'+str(y_min_grid)+')-> X('+str(agent_var[0])+'<'+str(y_min_grid)}
    return dynamics_spec

def check_circular(spec):
    if omega_intf.is_circular(spec):
        raise AssertionError('detected circularity in the specification')

def test_specs():
    '''
    Generate the intersection specs and print them.
    '''
    transitions_dict, actions_dict = make_state_dictionary_for_specification()
    test_spec, ego_spec = intersection_specs(transitions_dict)
    gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
    print(gr_spec.pretty())
    check_circular(gr_spec)
    synthesize_controller(gr_spec)

if __name__ == '__main__':
    test_specs()
