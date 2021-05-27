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
import omega
import tulip

class Spec:
    def __init__(self,vars,init,safety,progress):
        self.vars = vars
        self.init = init
        self.safety = safety
        self.progress = progress

class WinningSet:
    def __init__(self,test_spec, ego_spec):
        self.test_spec = test_spec
        self.ego_spec = ego_spec
        self.set = None

    def make_compatible_automaton(self):
        """
        create automaton from spec and abstraction
        @type spec: `tulip.spec.form.GRSpec`
        @type aut: `omega.symbolic.temporal.Automaton`
        """
        spec = spec.GRSpec(env_vars=env_vars, sys_vars=sys_vars,
                    env_init=env_init, sys_init=sys_init,
                    env_safety=env_safety, sys_safety=sys_safety,
                    env_prog=env_prog, sys_prog=sys_prog)
        aut = tulip._grspec_to_automaton(spec) #`temporal.Automaton` - compiled game with <>[] \/ []<> winning
        return aut


    def find_winning_set(self):
        # interface with TuLiP
        aut = self.make_compatible_automaton()
        z, yij, xijk = omega.solve_streett_game(aut)
        self.set = z
        pass

    def synthesize_shield(self):
        pass
