#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun  8 12:35:06 2021

@author: apurvabadithela, josefinegraebener
"""
# Ego: system
# Env: tester
# This file is to generate the GR1 specifications associated with the merge example abstraction
# Geometry:
#
#   x = 0  |1|2|3|4|5|  6
#y=1       | | | | | |
#y=2       |T| | | | |

from __future__ import print_function
import logging
import numpy as np
from tulip import spec, synth, hybrid, transys
from polytope import box2poly
from tulip.abstract import prop2part, discretize
from tulip.abstract.plot import plot_partition
from tulip.dumpsmach import write_python_case
from tulip.spec.gr1_fragment import response_to_gr1

def design_spec(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog):
    logging.basicConfig(level=logging.WARNING)
    show = False

    # Constructing GR1spec from environment and systems specifications:
    specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init,
                        env_safe, sys_safe, env_prog, sys_prog)
    return spec

def add_property(prop_set, new_prop):
    prop_set |= new_prop
    return prop_set

def var_setup():
    sys_vars = {}
    tester_vars = {}
    sys_vars['x'] = (0, 6) # 0: system is much behind the second tester car, 6: system is much further than the first tester car
    sys_vars['y'] = (1,2)
    tester_vars['x1'] = (1,6)
    tester_vars['y1'] = (1,2)
    tester_vars['x2'] = (0,5)
    tester_vars['y2'] = (1,2)
    return sys_vars, tester_vars

# T1 starts outside the box.
def init_setup():
    sys_init = {'x='+str(0), 'y='+str(1)}
    tester_init = {'x1='+str(1), 'y1='+str(2), 'x2='+str(0), 'y2='+str(2)}
    return sys_init, tester_init

def progress_prop():
    sys_prog = {'y=2'} # Eventually, the system should merge
    tester_prog = {} # Eventually, the tester cars should surround the system in lane 2
    for ki in range(1,4):
        tester_prog |= {'((x='+str(ki+1)+') && (x1='+str(ki+2)+') && (x2='+str(ki)+')) && (y=2 && y1=2 && y2=2)'}
    return sys_prog, tester_prog

# Once the system merges, it cannot merge outside the lane or move forward. The simulation ends
def append_final_prop(sys_safe, tester_safe):
    for xi in range(0,7):
        for x1i in range(1,7):
            for x2i in range(0,x1i):
                other_st_i = '((x = '+str(xi)+') && (x1='+str(x1i)+')&&(x2='+str(x2i)+'))'
                sys_safe |= {'((y=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2))'}
                tester_safe |= {'((y=2) && '+other_st_i+')-> X('+other_st_i+'&& (y=2))'}
    return sys_safe, tester_safe

def safe_props(sys_safe, tester_safe):
    tester_safe |= {'!(x1=x2 && y1=y2)'}
    sys_safe |= {'!(x1=x && y1=y)'}
    sys_safe |= {'!(x2=x && y2=y)'}
    tester_safe |= {'!(x1=x && y1=y)'}
    tester_safe |= {'!(x2=x && y2=y)'}

    # Testers never merge into lane 1 for simplicity:
    tester_safe |= {'!(y1=1) && !(y2=1)'}
    return sys_safe, tester_safe

def dyn_props(sys_safe, tester_safe):
    # Dynamics of tester 1 and tester 2 in the 5-cell grid:
    for ii in range(1,5):
        tester_safe |= {'(x1='+str(ii)+' && y1=1) -> X((x1='+str(ii+1)+' && y1=1)||(x1='+str(ii)+' && y1=1)|| (x1='+str(ii+1)+' && y1=2))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=2) -> X((x2='+str(ii+1)+' && y2=2)||(x2='+str(ii)+' && y2=2)|| (x2='+str(ii+1)+' && y2=1))'}
        tester_safe |= {'(x2='+str(ii)+' && y2=1) -> X((x2='+str(ii+1)+' && y2=1)||(x2='+str(ii)+' && y2=1)|| (x2='+str(ii+1)+' && y2=2))'}
        tester_safe |= {'(x1='+str(ii)+' && y1=2) -> X((x1='+str(ii+1)+' && y1=2)||(x1='+str(ii)+' && y1=2)|| (x1='+str(ii+1)+' && y1=1))'}
    for ii in range(1,5):
        sys_safe |= {'(x='+str(ii)+' && y=1) -> X((x='+str(ii+1)+' && y=1)||(x='+str(ii)+' && y=1)|| (x='+str(ii+1)+' && y=2))'}
        sys_safe |= {'(x='+str(ii)+' && y=2) -> X((x='+str(ii+1)+' && y=2)||(x='+str(ii)+' && y=2)|| (x='+str(ii+1)+' && y=1))'}
    return sys_safe, tester_safe

# End cases properties appending to existing safety properties:
# Atleast one of the testers has to be on the board. Therefore, T1 has positions ranging from 1 to 6,
# and T2 has positions ranging from 0 to 5.
def safe_append(sys_safe, tester_safe):
    # Atleast one tester needs to be on the 5-cell board: Cannot have x2=6 and x1=0 at the same time
    tester_safe |= {'!(x1=6 && x2=0)'}

    # T2 at x=5 remains at x=5:
    tester_safe |= {'(x2=5 -> X(x2=5)'}
    # T1 at x=6 remains at x=6:
    tester_safe |= {'(x1=6 -> X(x1=6)'}

    # T1 at x=5 can stay or transition:
    tester_safe |= {'(x1=5 -> X(x1=5 || x1=6)'}

    # T2 at x=0 can stay or transition:
    tester_safe |= {'(x2=0 -> X(x2=0 || x2=1)'}
    # System states append:
    sys_safe |= {'x=0 -> X(x=0 && x=1)'}
    sys_safe |= {'x=5 -> X(x=5 && x=6)'}
    sys_safe |= {'x=6 -> X(x=6)'}

    return sys_safe, tester_safe


def gen_spec():
    sys_safe = {}
    tester_safe = {}

    sys_vars, tester_vars = var_setup()
    sys_init, tester_init = init_setup()
    sys_prog, tester_prog = progress_prop()
    sys_safe, tester_safe = safe_props(sys_safe, tester_safe)
    sys_safe, tester_safe = safe_append(sys_safe, tester_safe)
    sys_safe, tester_safe = dyn_props(sys_safe, tester_safe)
    sys_safe, tester_safe = append_final_prop(sys_safe, tester_safe)

    return sys_vars, tester_vars, sys_init, tester_init, sys_safe, tester_safe, sys_prog, tester_prog


def spec_pipeline():
    sys_vars, tester_vars, sys_init, tester_init, sys_safe, tester_safe, sys_prog, tester_prog = gen_spec()
    spec = design_spec(sys_vars, tester_vars, sys_init, tester_init, sys_safe, tester_safe, sys_prog, tester_prog)
    return spec
