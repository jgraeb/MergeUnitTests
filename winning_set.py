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
        self.winning_set = None

    def make_compatible_automaton(self):
        """
        create automaton from spec and abstraction
        @type spec: `tulip.spec.form.GRSpec`
        @type aut: `omega.symbolic.temporal.Automaton`
        """
        spec = spec.GRSpec(env_vars=self.ego_spec.vars, sys_vars=self.test_spec.vars,
                    env_init=self.ego_spec.init, sys_init=self.test_spec.init,
                    env_safety=self.ego_spec.safety, sys_safety=self.test_spec.safety,
                    env_prog=self.ego_spec.prog, sys_prog=self.test_spec.prog)
        aut = tulip._grspec_to_automaton(spec) #`temporal.Automaton` - compiled game with <>[] \/ []<> winning
        return aut


    def find_winning_set(self):
        # interface with TuLiP
        aut = self.make_compatible_automaton()
        z, yij, xijk = omega.solve_streett_game(aut)
        self.winning_set = z


    def synthesize_shield(self):
        # create dictionary of allowable actions per state
        pass

if __name__ == '__main__':
    #testing winning set computation
    # define the specs here
    test_spec = Spec()
    ego_spec = Spec()
    w_set = WinningSet(test_spec,ego_spec)
    w_set.find_winning_set()
    st()
