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

def check_circular(spec):
    if omega_intf.is_circular(spec):
        raise AssertionError('detected circularity in the specification')