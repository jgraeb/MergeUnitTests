## Apurva Badithela
# Oct 21, 2021
from __future__ import print_function
from copy import deepcopy
import logging
import os
import _pickle as pickle
from random import choice
import sys
print(sys.path)
sys.path.append('..')
sys.path.append('../tulip-control/')
sys.path.append('../omega/')

import networkx as nx
import numpy as np
# from ipdb import set_trace as st
from omega.games import enumeration as enum
from omega.games import gr1
from omega.logic import syntax as stx
from omega.symbolic import enumeration as sym_enum
from omega.symbolic import fixpoint as fx
from omega.symbolic import fol as _fol
from omega.symbolic import prime as prm
from omega.symbolic import temporal as trl
import pdb
# from scene import Scene
from tulip.interfaces.omega import (
    _grspec_to_automaton, _strategy_to_state_annotated)
from tulip import spec
from tulip.spec import form
from tulip import synth
from tulip.synth import sys_to_spec


# Dynamics of static agents on a map:
def dynamic_transitions(agent_vars: List[str, str], map: Dict[List], agent_safety_spec: set):
    xpos = agent_vars[0]
    ypos = agent_vars[1]
    for k, val in map.items():
        xpos_i = k[0]
        ypos_i = k[1]
        assumption = '('+xpos+rf'''={xpos_i} /\ '''+ ypos+rf'''={ypos_i}) => '''
        guarantee = ""
        for vi in val:
            xpos_n = vi[0]
            ypos_n = vi[1]
            guar_vi = '('+xpos+rf'''={xpos_n} /\ '''+ ypos+rf'''={ypos_n})'''
            if guarantee == "":
                guarantee += "(" + guar_vi
            else:
                guarantee += "\/" + guar_vi
        guarantee += ")"

        agent_safety_spec |= {assumption + guarantee} # Appending to the set
