#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append('..')
import numpy as np
import pdb
import logging
from tulip import transys, spec, synth
from tulip.interfaces.omega import _grspec_to_automaton
import tulip.interfaces.omega as omega_intf
from intersection import make_state_dictionary_for_specification
from tools import create_intersection_from_file
from spec_tools import Spec, make_grspec, check_circular

def collision_safety(y_min_grid, y_max_grid, z_min_grid, z_max_grid, crosswalk):
    '''
    Create the specifications to ensure no collisions between the agents.
    '''
    tester_safe = set()
    sys_safe = set()
    # No collision with other vehicles:
    for zi in range(z_min_grid,z_max_grid):
        for yi in range(y_min_grid, y_max_grid):
            for loc in crosswalk.keys():
                cw_num = []
                if crosswalk[loc] == (yi, zi):
                    cw_num.append(loc)
                ped_string = ''
                for item in cw_num:
                    if ped_string == '':
                        ped_string = ped_string + '(p= '+str(item)+')'
                    else:
                        ped_string = ped_string + ' || (p= '+str(item)+')'

            tester_safe |= {'!(y1='+str(yi)+' && z1 ='+str(zi)+' && '+str(ped_string)+')'}
            tester_safe |= {'(y='+str(yi)+' && z= '+str(zi)+') -> X(!(y1 ='+str(yi)+'&& z1 = '+str(zi)+'))'}
            tester_safe |= {'(y='+str(yi)+' && z= '+str(zi)+') -> X(!('+str(ped_string)+'))'}
            sys_safe |= {'(y1 ='+str(yi)+' && z1 = '+str(zi)+') -> X(!(y='+str(yi)+' && z= '+str(zi)+ '))'}
            sys_safe |= {'('+str(ped_string)+') -> X(!(y='+str(yi)+' && z = '+str(zi)+ '))'}
    return tester_safe, sys_safe

def dynamics_car(state_dict, agent_var_list, y_min_grid, y_max_grid, z_min_grid, z_max_grid):
    '''
    Add the dynamics from the intersection grid to the specifications.
    '''
    dynamics_spec = set()
    for agent_var in agent_var_list:
        for ii in range(y_min_grid,y_max_grid):
            for jj in range(z_min_grid,z_max_grid):
                if (ii,jj) in state_dict:
                    next_steps_string = ''
                    for item in state_dict[(ii,jj)]:
                        if next_steps_string == '':
                            next_steps_string = next_steps_string + '('+str(agent_var[0])+'='+str(item[0])+' && '+str(agent_var[1])+' ='+str(item[0])+')'
                        else:
                            next_steps_string = next_steps_string + ' || ('+str(agent_var[0])+'='+str(item[0])+' && '+str(agent_var[1])+'='+str(item[0])+')'
                    dynamics_spec |= {'('+str(agent_var[0])+'='+str(ii)+' && '+str(agent_var[1])+'='+str(jj)+') -> X(('+ next_steps_string +'))'}
    return dynamics_spec

def dynamics_ped(ped_var, min_cw, max_cw):
    '''
    Add the dynamics of the pedestrian agent.
    '''
    dynamics_ped = set()
    for cw_loc in range(min_cw, max_cw):
        next_steps_string = '('+str(ped_var)+'='+str(cw_loc)+' || '+str(ped_var)+'='+str(cw_loc + 1)+')'
        dynamics_ped |= {'('+str(ped_var)+'='+str(cw_loc)+') -> X(('+ next_steps_string +'))'}
    cw_loc = max_cw
    dynamics_ped |= {'('+str(ped_var)+'='+str(cw_loc)+') -> X(('+str(ped_var)+'='+str(cw_loc)+'))'}
    return dynamics_ped

def intersection_specs(state_dict, crosswalk):
    # grid variables
    y_min_grid = 0
    y_max_grid = 7
    z_min_grid = 0
    z_max_grid = 7
    min_cw = 0
    max_cw = 7

    # bounds = [x_min_grid,x_max_grid, y_min_grid,y_max_grid]
    y_init = 7
    z_init = 4
    y_goal = 3
    z_goal = 0
    sys_vars = {}
    sys_vars['y'] = (y_min_grid, y_max_grid)
    sys_vars['z'] = (z_min_grid, z_max_grid)
    sys_init = {'y='+str(y_init)+' && z='+str(z_init)}
    sys_prog = set()
    sys_prog |= {'y='+str(y_goal)+' && z='+str(z_goal)}
    sys_safe = set()
    # add the dynamics for the system
    sys_safe |= dynamics_car(state_dict,[('y','z')], y_min_grid,y_max_grid, z_min_grid,z_max_grid)

    # tester car + pedestrian
    # initial positions
    y1_init = 0
    z1_init = 3
    ped_init = 0
    tester_vars = {}
    # set initial conditions
    tester_vars['y1'] = (y_min_grid, y_max_grid)
    tester_vars['z1'] = (z_min_grid, z_max_grid)
    tester_vars['p'] = (min_cw, max_cw)
    tester_init = {'y1='+str(y1_init), 'z1='+str(z1_init), 'p='+str(0)}
    tester_prog = set()
    tester_safe = set()

    # Add the dynamics
    tester_safe |= dynamics_car(state_dict, [('y1','z1')], y_min_grid, y_max_grid, z_min_grid, z_max_grid)
    tester_safe |= dynamics_ped('p', min_cw, max_cw)

    # Add no collissions between any agents
    no_collisions_tester, no_collisions_sys = collision_safety(y_min_grid, y_max_grid, z_min_grid, z_max_grid, crosswalk)
    tester_safe |= no_collisions_tester
    sys_safe |= no_collisions_sys

    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    return ego_spec, test_spec

def test_specs():
    '''
    Generate the intersection specs and print them.
    '''
    intersectionfile = 'intersectionfile.txt'
    map, crosswalk = create_intersection_from_file(intersectionfile)
    # transitions_dict, actions_dict = make_state_dictionary_for_specification()
    test_spec, ego_spec = intersection_specs(map, crosswalk)
    gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
    print(gr_spec.pretty())
    # check_circular(gr_spec)
    # synthesize_controller(gr_spec)

if __name__ == '__main__':
    test_specs()
