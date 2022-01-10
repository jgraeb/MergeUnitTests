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
from spec_tools import Spec, make_grspec, check_circular, check_specs
from graph_construction import find_next_state_dict
from ipdb import set_trace as st

def collision_safety(y_min_grid, y_max_grid, z_min_grid, z_max_grid, crosswalk):
    '''
    Create the specifications to ensure no collisions between the agents.
    '''
    tester_safe = set()
    sys_safe = set()
    # No collision with other vehicles:
    for zi in range(z_min_grid,z_max_grid):
        for yi in range(y_min_grid, y_max_grid):
            cw_num = []
            for loc in crosswalk.keys():
                # st()
                if crosswalk[loc] == (yi, zi):
                    cw_num.append(loc)
                ped_string = ''
                for item in cw_num:
                    if ped_string == '':
                        ped_string = ped_string + '(p = '+str(item)+')'
                    else:
                        ped_string = ped_string + ' || (p = '+str(item)+')'

            tester_safe |= {'(y = '+str(yi)+' && z = '+str(zi)+') -> X(!(y1 = '+str(yi)+' && z1 = '+str(zi)+'))'}
            sys_safe |= {'(y1 = '+str(yi)+' && z1 = '+str(zi)+') -> X(!(y = '+str(yi)+' && z = '+str(zi)+ '))'}

            if not ped_string == '':
                tester_safe |= {'!(y1 = '+str(yi)+' && z1 = '+str(zi)+' && '+str(ped_string)+')'}
                tester_safe |= {'(y = '+str(yi)+' && z = '+str(zi)+') -> X(!('+str(ped_string)+'))'}
                sys_safe |= {'('+str(ped_string)+') -> X(!(y='+str(yi)+' && z = '+str(zi)+ '))'}


            # check for pedestrian
    return tester_safe, sys_safe

def dynamics_system(state_dict, agent_var_list, y_min_grid, y_max_grid, z_min_grid, z_max_grid):
    '''
    Add the dynamics from the intersection grid to the specifications.
    '''
    next_state_dict = find_next_state_dict(state_dict)
    dynamics_spec = set()
    for agent_var in agent_var_list:
        for ii in range(y_min_grid,y_max_grid):
            for jj in range(z_min_grid,z_max_grid):
                if not state_dict[(ii,jj)] == '*':
                    next_steps_string = ''
                    # always add current state
                    next_steps_string = next_steps_string + '('+str(agent_var[0])+' = '+str(ii)+' && '+str(agent_var[1])+' = '+str(jj)+')'
                    if state_dict[(ii,jj)] == '↑':
                        next_steps_string = next_steps_string + '|| ('+str(agent_var[0])+' = '+str(ii-1)+' && '+str(agent_var[1])+' = '+str(jj)+')'
                    elif state_dict[(ii,jj)] == '←':
                        next_steps_string = next_steps_string + '|| ('+str(agent_var[0])+' = '+str(ii)+' && '+str(agent_var[1])+' = '+str(jj-1)+')'
                    elif state_dict[(ii,jj)] == '+':
                        if ii == 4:
                            next_steps_string = next_steps_string + '|| ('+str(agent_var[0])+' = '+str(ii-1)+' && '+str(agent_var[1])+' = '+str(jj)+')'
                        elif ii == 3:
                            next_steps_string = next_steps_string + '|| ('+str(agent_var[0])+' = '+str(ii)+' && '+str(agent_var[1])+' = '+str(jj-1)+')'

                    dynamics_spec |= {'('+str(agent_var[0])+' = '+str(ii)+' && '+str(agent_var[1])+' = '+str(jj)+') -> X(('+ next_steps_string +'))'}
    return dynamics_spec

def dynamics_tester_car(state_dict, agent_var, y_min_grid, y_max_grid, z_min_grid, z_max_grid):
    '''
    Add the dynamics from the intersection grid to the specifications.
    '''
    jj = 3 # tester car is only in z = 3 lane
    dynamics_spec = set()
    for ii in range(y_min_grid,y_max_grid):
        if not state_dict[(ii,jj)] == '*':
            next_steps_string = ''
            # always add current state
            next_steps_string = next_steps_string + '('+str(agent_var[0])+' = '+str(ii)+' && '+str(agent_var[1])+' = '+str(jj)+')'
            if state_dict[(ii,jj)] == '↓':
                next_steps_string = next_steps_string + '|| ('+str(agent_var[0])+' = '+str(ii+1)+' && '+str(agent_var[1])+' = '+str(jj)+')'

            dynamics_spec |= {'('+str(agent_var[0])+' = '+str(ii)+' && '+str(agent_var[1])+' = '+str(jj)+') -> X(('+ next_steps_string +'))'}
    return dynamics_spec

def dynamics_ped(ped_var, min_cw, max_cw):
    '''
    Add the dynamics of the pedestrian agent.
    '''
    dynamics_ped = set()
    for cw_loc in range(min_cw, max_cw):
        next_steps_string = '('+str(ped_var)+' = '+str(cw_loc)+' || '+str(ped_var)+' = '+str(cw_loc + 1)+')'
        dynamics_ped |= {'('+str(ped_var)+' = '+str(cw_loc)+') -> X(('+ next_steps_string +'))'}
    cw_loc = max_cw
    dynamics_ped |= {'('+str(ped_var)+' = '+str(cw_loc)+') -> X(('+str(ped_var)+' = '+str(cw_loc)+'))'}
    return dynamics_ped

#   Car drives afte left turn
# Add
def once_system_entered_intersection_keep_driving(state_dict, agent_var, y_val, z_min, z_max):
    '''
    Once the system is in cell (3,4) it must not stop anymore.
    '''
    z_min = 0
    z_max = 4 # we only care about the states up to 4 in z direction for the system
    # next_state_dict = find_next_state_dict(state_dict)
    safety_spec = set()
    for jj in range(z_min,z_max+1):
        if not state_dict[(y_val,jj)] == '*':
            next_state_string = ''
            if jj == 0:
                next_state_string = next_state_string + '('+str(agent_var[0])+' = '+str(y_val)+' && '+str(agent_var[1])+' = '+str(jj)+')'
            else:
                next_state_string = next_state_string + '('+str(agent_var[0])+' = '+str(y_val)+' && '+str(agent_var[1])+' = '+str(jj-1)+')'
            safety_spec |= {'('+str(agent_var[0])+' = '+str(y_val)+' && '+str(agent_var[1])+' = '+str(jj)+') -> X(('+ next_state_string +'))'}
    return safety_spec

# Add!
def intersection_clear_eventually_system_drives(state_dict, y_min_grid, y_max_grid, z_min_grid, z_max_grid):
    '''
    Once the intersection is free, the car must go.
    '''
    tester_car_not_intersection_states = [(4,3), (5,3), (6,3), (7,3)]
    tester_pedestrian_not_crosswalk_states = [6,7,8,9]
    system_states = [(4,4), (5,4), (6,4), (7,4)]
    safety_spec = set()
    for sys_state in system_states:
        next_state_string = '( y = '+str(sys_state[0] - 1)+' && z = '+str(sys_state[1])+')' # move up one cell
        for tester_car in tester_car_not_intersection_states:
            for ped in tester_pedestrian_not_crosswalk_states:
                safety_spec |= {'(y = '+str(sys_state[0])+' && z = '+str(sys_state[1])+' && y1 = '+str(tester_car[0])+' && z1 = '+str(tester_car[1])+' && p = '+str(ped)+') -> X(('+ next_state_string +'))'}
    return safety_spec

# Variables:
def sys_variables():
    y_min_grid = 3
    y_max_grid = 7
    z_min_grid = 0
    z_max_grid = 4
    sys_vars = {}
    # bounds = [x_min_grid,x_max_grid, y_min_grid,y_max_grid]
    sys_vars['y'] = (y_min_grid, y_max_grid)
    sys_vars['z'] = (z_min_grid, z_max_grid)
    return sys_vars, y_min_grid, y_max_grid, z_min_grid, z_max_grid


def tester_variables(y_min_grid, y_max_grid, z_min_grid, z_max_grid):
    min_cw = 0
    max_cw = 9
    tester_vars = {}
    # set initial conditions
    tester_vars['y1'] = (y_min_grid, y_max_grid)
    tester_vars['z1'] = (z_min_grid, z_max_grid)
    tester_vars['p'] = (min_cw, max_cw)
    return tester_vars, min_cw, max_cw

# Initial conditions for system:
def initial_sys_vars():
    sys_init = set()
    y_init = 7
    z_init = 4
    sys_init = {'y='+str(y_init)+' && z='+str(z_init)}
    return sys_init

def init_tester_vars():
    y1_init = 0
    z1_init = 3
    ped_init = 0
    tester_init = {'y1='+str(y1_init), 'z1='+str(z1_init), 'p='+str(0)}
    return tester_init

# Progress guarantee for system:
def progress_sys_vars():
    sys_prog = set()
    y_goal = 3
    z_goal = 0
    sys_prog |= {'y='+str(y_goal)+' && z='+str(z_goal)}
    return sys_prog

# Function to add auxiliary tester specifications:
def auxiliary_tester_specs(tester_vars, tester_init, tester_safe, tester_prog):
    tester_vars['wflg'] = (0,1)
    tester_vars['pflg'] = (0,1)
    tester_vars['tflg'] = (0,1)
    tester_init |= {'wflg=0', 'pflg=1', 'tflg=1'}

    tester_safe |= add_prog_flg_specs(spec="wait_for_car")
    tester_safe |= add_prog_flg_specs(spec="wait_for_ped")
    tester_safe |= add_prog_flg_specs(spec="reach_goal")

    tester_prog |= add_progress_specs(spec="wait_for_car")
    tester_prog |= add_progress_specs(spec="wait_for_ped")
    tester_prog |= add_progress_specs(spec="reach_goal")
    return tester_vars, tester_init, tester_safe, tester_prog

def intersection_specs(state_dict, crosswalk):
    # grid variables
    sys_vars, y_min_grid, y_max_grid, z_min_grid, z_max_grid = sys_variables()
    sys_init = initial_sys_vars()
    sys_prog = progress_sys_vars()
    sys_safe = set()

    # add the dynamics for the system
    sys_safe |= dynamics_system(state_dict,[('y','z')], y_min_grid,y_max_grid, z_min_grid,z_max_grid)
    sys_safe |= intersection_clear_eventually_system_drives(state_dict, y_min_grid, y_max_grid, z_min_grid, z_max_grid)
    y_val = 3
    z_min = 0
    z_max = 7
    agent_var = ['y','z']
    sys_safe |= once_system_entered_intersection_keep_driving(state_dict, agent_var, y_val, z_min, z_max)
    # tester car + pedestrian
    # initial positions
    y_min_t = 3
    y_min_t = 3
    z_min_t = 0
    z_max_t = 7
    tester_vars, min_cw, max_cw = tester_variables(y_min_grid, y_max_grid, z_min_grid, z_max_grid)
    tester_init = init_tester_vars()

    tester_prog = set()
    tester_safe = set()

    # Add the dynamics
    tester_safe |= dynamics_tester_car(state_dict, ('y1','z1'), y_min_grid, y_max_grid, z_min_grid, z_max_grid)
    tester_safe |= dynamics_ped('p', min_cw, max_cw)

    # Add no collissions between any agents
    no_collisions_tester, no_collisions_sys = collision_safety(y_min_grid, y_max_grid, z_min_grid, z_max_grid, crosswalk)
    tester_safe |= no_collisions_tester
    sys_safe |= no_collisions_sys

    # Progress auxiliary specs
    tester_vars, tester_init, tester_safe, tester_prog = auxiliary_tester_specs(tester_vars, tester_init, tester_safe, tester_prog)

    ego_spec = Spec(sys_vars, sys_init, sys_safe, sys_prog)
    test_spec = Spec(tester_vars, tester_init, tester_safe, tester_prog)
    return ego_spec, test_spec

# Add safety specifications that correspond to the progress:
def add_prog_flg_specs(spec=spec):
    safety_spec = set()
    if spec == "wait_for_car":
        ywait = 4
        zwait = 4
        xinter = [(1,3),(2,3),(3,3)]
        for yc, zc in xinter:
            safety_spec |= {'(y = '+str(ywait)+' && z='+str(zwait) + ' && y1 = '+str(yc)+' && z1 = ' + str(yc) + ' && wflg = 0) -> X(wflg = 1)'}
        safety_spec |= {'(wflg = 1) -> X(wflg = 1)'}

    # Adding specifications for waiting for pedestrian:
    elif spec == "wait_for_ped":
        ywait = 4
        zwait = 4
        xcrsng = [0,1,2,3]
        for p in xcrsng:
            safety_spec |= {'(y = '+str(ywait)+' && z='+str(zwait) + ' && p = '+str(p)+ ' && pflg = 0) -> X(pflg = 1)'}
            safety_spec |= {'(pflg = 1) -> X(pflg = 1)'}

    elif spec == "reach_goal":
        yturn = 3
        zturn = 1
        safety_spec |= {'(y = '+str(yturn)+' && z='+str(zturn) + ' && tflg = 0) -> X(tflg = 1)'}
        safety_spec |= {'(tflg = 1) -> X(tflg = 1)'}
    else:
        print("Not correct spec type")
    return safety_spec

# Add progress specifications:
def add_progress_specs(spec=spec):
    prog_spec = set()
    # Adding specifications for waiting for Car
    if spec == "wait_for_car":
        prog_spec |= {'(wflg=1)'}

    # Adding specifications for waiting for pedestrian:
    elif spec == "wait_for_ped":
        prog_spec |= {'(pflg=1)'}

    elif spec == "reach_goal":
        prog_spec |= {'(tflg=1)'}
    else:
        print("Not correct spec type")
        pdb.set_trace()
    return prog_spec

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
    check_specs(gr_spec)
    # check_circular(gr_spec)
    # synthesize_controller(gr_spec)

if __name__ == '__main__':
    test_specs()
