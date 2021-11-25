
#!/usr/bin/env python3
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

class Spec:
    def __init__(
            self, sys_vars,
            init, safety, progress):
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
    return spec.GRSpec(
        env_vars, sys_vars,
        env_init, sys_init,
        env_safe, sys_safe,
        env_prog, sys_prog)


class WinningSet:
    def __init__(self):
        self.spec = None
        self.winning_set = None

    def make_compatible_automaton(self, spec):
        """Create automaton from `spec` and abstraction.

        opt = tester (test environment needs a controller) or
        system (system needs a controller)

        @type spec: `tulip.spec.form.GRSpec`
        @rtype: `omega.symbolic.temporal.Automaton`
        """
        # compiled game with `<>[] \/ []<>` winning
        return _grspec_to_automaton(spec)

    def check_state_in_winset(self, aut, z, state_dict):
        """Check whether state can be in winning set.

        Inputs:
        - automaton `aut`, winning set BDD `z`
        - state dictionary `state_dict`

        The state dictionary should be as follows:
        `state_dict = {'x': 10, 'y':15}`, if the state
        we want to check has values of `x = 10` and `y = 15`.
        """
        check_bdd = aut.let(state_dict, z)
        if check_bdd.var is not None:
            raise RuntimeError(
                'expected constant, '
                f'computed: {aut.bdd.to_expr(check_bdd)}')
        return check_bdd == aut.true

    def find_winning_set(self, aut):
        """Interface with TuLiP."""
        z, yij, xijk = gr1.solve_streett_game(aut)
        self.winning_set = z
        return z

    def synthesize_shield(self):
        """Allowable actions per state.

        For now manually -> should be from winning set.

        @rtype: `dict`
        """
        shield_dict = dict()
        shield_dict.update(s0_t='move')
        shield_dict.update(s1_t=['move', 'stay'])
        shield_dict.update(s2_t=['stay'])
        shield_dict.update(s5_t=['move'])
        # print(shield_dict)
        return shield_dict

    def get_winning_set_shield(self, w_set):
        fsm = w_set.make_labeled_fsm()
        spec = w_set.spec_from_fsm(fsm)
        winning_set = w_set.find_winning_set(spec)
        shield_dict = w_set.synthesize_shield()
        return shield_dict

def specs_two_testers(tracklength):
    """Specifications for automaton."""
    # st()
    sys_vars = dict()
    sys_vars['x'] = (1, tracklength)
    sys_vars['y'] = (1, 2)
    sys_init = {'x = 1', 'y = 1'}
    sys_prog = {'y = 2'}
        # Eventually, the system should merge
    sys_safe = set()
    # Dynamics for merging into adjacent track:
    for ii in range(1, tracklength):
        sys_safe |= {
            rf'''
            (x = {ii} /\ y = 1) => (
                   (x = {ii + 1} /\ y = 1)
                \/ (x = {ii} /\ y = 1)
                \/ (x = {ii + 1} /\ y = 2)'
            '''}
        sys_safe |= {
            rf'''
            (x = {ii} /\ y = 2) => (
                   (x = {ii + 1} /\ y = 2)
                \/ (x = {ii} /\ y = 2)
                \/ (x = {ii + 1} /\ y = 1)'
            '''}
        # Pause system after merge
        # sys_safe |= {
        #     rf"(x = {ii} /\ y = 2) => (x = {ii} /\ y = 2)' "}
    # sys_safe |= {r" x = 0 => (x = 0 /\ x = 1)' "}
    # Dynamics of the system in the last cell:
    sys_safe |= {
        rf'''
        (x = {tracklength} /\ y = 1)
        => (x = {tracklength} /\ y = 1)'
        '''}
    sys_safe |= {
        rf'''
        (x = {tracklength} /\ y = 2)
        => (x = {tracklength} /\ y = 2)'
        '''}
    # testers
    tester_vars = dict()
    tester_vars['x1'] = (2, tracklength)
    tester_vars['y1'] = (1, 2)
    tester_vars['x2'] = (1, tracklength - 1)
    tester_vars['y2'] = (1, 2)
    tester_init = {
        'x1 = 3',
        'y1 = 2',
        'x2 = 2',
        'y2 = 2'}
    tester_prog = set()
    merge_spec = r'''
        (  x = 2 /\ x1 = 3 /\ x2 = 2
        /\ y = 2 /\ y1 = 2 /\ y2 = 2)
        '''
    # Freeze final specs:
    tester_safe = set()
    # tester_safe |= {f" {merge_spec} => ({merge_spec})' "}
    # sys_safe |= {f" {merge_spec} => ({merge_spec})' "}
    # sys_merge_spec = r'(x = 2 /\ y = 2)'
    for ki in range(2, tracklength - 1):
        new_merge_state = (
            rf'(x = {ki + 1} /\ x1 = {ki + 2}'
            rf' /\ x2 = {ki} /\ y = 2 /\ y1 = 2 /\ y2 = 2)')
        # tester_safe |= {
        #     rf"{new_merge_state} => ({new_merge_state})' "}
        # sys_safe |= {
        #     rf"{new_merge_state} => ({new_merge_state})' "}
        merge_spec += rf' \/ {new_merge_state}'
        # sys_merge_spec += rf' \/ (y = 2 /\ x = {ki + 1})'
    tester_prog |= {merge_spec}
   
    tester_safe |= {'y1 = 2'}
        # testers stay in bottom lane
    tester_safe |= {'y2 = 2'}
    # Tester dynamics
    for ii in range(2, tracklength):
        tester_safe |= {
            rf'''
            (x1 = {ii} /\ y1 = 1) => (
                   (x1 = {ii + 1} /\ y1 = 1)
                \/ (x1 = {ii} /\ y1 = 1)
                \/ (x1 = {ii + 1} /\ y1 = 2))'
            '''}
        tester_safe |= {
            rf'''
            (x1 = {ii} /\ y1 = 2) => (
                   (x1 = {ii + 1} /\ y1 = 2)
                \/ (x1 = {ii} /\ y1 = 2)
                \/ (x1 = {ii + 1} /\ y1 = 1)')
            '''}
    for ii in range(1, tracklength):
        if ii != tracklength - 1:
            tester_safe |= {
                rf'''
                (x2 = {ii} /\ y2 = 2) => (
                       (x2 = {ii + 1} /\ y2 = 2)
                    \/ (x2 = {ii} /\ y2 = 2)
                    \/ (x2 = {ii + 1} /\ y2 = 1))'
                '''}
            tester_safe |= {
                rf'''
                (x2 = {ii} /\ y2 = 1) => (
                       (x2 = {ii + 1} /\ y2 = 1)
                    \/ (x2 = {ii} /\ y2 = 1)
                    \/ (x2 = {ii + 1} /\ y2 = 2))'
                '''}
            # print('skip')
        else:
            tester_safe |= {
                rf'''
                (x2 = {ii} /\ y2 = 2)
                => (x2 = {ii} /\ y2 = 2)'
                '''}
            tester_safe |= {
                rf'''
                (x2 = {ii} /\ y2 = 1)
                => (x2 = {ii} /\ y2 = 1)'
                '''}
    tester_safe |= {
        rf'''
        (x1 = {tracklength} /\ y1 = 2)
        => (x1 = {tracklength} /\ y1 = 2)'
        '''}
    tester_safe |= {
        rf'''
        (x1 = {tracklength} /\ y1 = 1)
        => (x1 = {tracklength} /\ y1 = 1)'
        '''}

    # Synthesize specs
    ego_spec = Spec(
        sys_vars, sys_init,
        sys_safe, sys_prog)
    test_spec = Spec(
        tester_vars, tester_init,
        tester_safe, tester_prog)
    # st()
    return ego_spec, test_spec


# System and environment have been switched:
def specs_for_entire_track(tracklength):
    # st()
    sys_vars = dict()
    sys_vars['x'] = (1, tracklength)
    sys_vars['y'] = (1, 2)
    sys_init = {'x = 1', 'y = 1'}
    sys_prog = {'y = 2'}
        # Eventually, the system should merge
    sys_safe = set()
    # Dynamics for merging into adjacent track:
    for ii in range(1, tracklength):
        sys_safe |= {
            rf'''
            (x = {ii} /\ y = 1) => (
                   (x = {ii + 1} /\ y = 1)
                \/ (x = {ii} /\ y = 1)
                \/ (x = {ii + 1} /\ y = 2))'
            '''}
        sys_safe |= {
            rf'''
            (x = {ii} /\ y = 2) => (
                   (x = {ii + 1} /\ y = 2)
                \/ (x = {ii} /\ y = 2)
                \/ (x = {ii + 1} /\ y = 1))'
            '''}
    # sys_safe |= {r" x = 0 => (x = 0 /\ x = 1)' "}
    sys_safe |= {
        rf'''
        x = {tracklength - 1} =>
            (x = {tracklength - 1} /\ x = {tracklength})'
        '''}
    sys_safe |= {
        rf" x ={tracklength} /\ y=1 => (x = {tracklength} /\ y=1)' "}
    sys_safe |= {
        rf" x ={tracklength} /\ y=2 => (x = {tracklength} /\ y=2)' "}
    
    # testers
    tester_vars = dict()
    tester_vars['x1'] = (2, tracklength)
    tester_vars['y1'] = (1, 2)
    tester_vars['x2'] = (1, tracklength - 1)
    tester_vars['y2'] = (1, 2)
    tester_init = {
        'x1 = 2',
        'y1 = 2',
        'x2 = 1',
        'y2 = 2'}
    tester_prog = set()
    merge_spec = r'''
        (  x = 2 /\ x1 = 3 /\ x2 = 1
        /\ y = 2 /\ y1 = 2 /\ y2 = 2)
        '''
    for ki in range(2, tracklength - 1):
        merge_spec += rf'''
            \/ (
                (x = {ki + 1})
                /\ (x1 = {ki + 2})
                /\ (x2 = {ki})
                /\ (y = 2 /\ y1 = 2 /\ y2 = 2))
            '''
    tester_prog |= {merge_spec}
    tester_safe = set()
    # No collision with other vehicles:
    for yi in range(1, 3):
        for xi in range(1, tracklength + 1):
            if xi != tracklength:
                tester_safe |= {
                    rf'''
                    ~ (   x1 = {xi} /\ x2 = {xi}
                       /\ y1 = {yi} /\ y2 = {yi})
                    '''}
                tester_safe |= {
                    rf'''
                    ~ (   x = {xi} /\ x2 = {xi}
                       /\ y = {yi} /\ y2 = {yi})
                    '''}
                sys_safe |= {
                    rf'''
                    ~ (   x = {xi} /\ x2 = {xi}
                       /\ y = {yi} /\ y2 = {yi})
                    '''}
            if xi != 1:
                tester_safe |= {
                    rf'''
                    ~ (   x1 = {xi} /\ x2 = {xi}
                       /\ y1 = {yi} /\ y2 = {yi})
                    '''}

                tester_safe |= {
                    rf'''
                    ~ (   x = {xi} /\ x1 = {xi}
                       /\ y = {yi} /\ y1 = {yi})
                    '''}
                sys_safe |= {
                    rf'''
                    ~ (   x = {xi} /\ x1 = {xi}
                       /\ y = {yi} /\ y1 = {yi})
                    '''}
    sys_safe |= {
        r' ~ (x = 1 /\ x2 = 1 /\ y= 2 /\ y2 = 2)'}
    sys_safe |= {
        r' ~ (x = 1 /\ x2 = 1 /\ y= 1 /\ y2 = 1)'}
    sys_safe |= {
        rf'''
        ~ (x = {tracklength} /\ x1 = {tracklength}
           /\ y = 2 /\ y1 = 2)
        '''}
    sys_safe |= {
        rf'''
        ~ (x = {tracklength} /\ x1 = {tracklength}
           /\ y = 1 /\ y1 = 1)
        '''}
    tester_safe |= {
        rf' ~ (x = 1 /\ x2 = 1 /\ y= 2 /\ y2 = 2) '}
    tester_safe |= {
        rf'''
        ~ (x = {tracklength} /\ x1 = {tracklength}
            /\ y = 2 /\ y1 = 2)
        '''}
    tester_safe |= {
        r' ~ (x = 1 /\ x2 = 1 /\ y = 1 /\ y2 = 1) '}
    tester_safe |= {
        rf'''
        ~ (x = {tracklength} /\ x1 = {tracklength}
           /\ y = 1 /\ y1 = 1)
        '''}
    tester_safe |= {r' ~ (y1 = 1) /\ ~ (y2 = 1) '}
        # testers stay in bottom lane
    # Tester dynamics
    for ii in range(2, tracklength - 1):
        tester_safe |= {
            rf'''
            (x1 = {ii} /\ y1 = 1) => (
                   (x1 = {ii + 1} /\ y1 = 1)
                \/ (x1 = {ii} /\ y1 = 1)
                \/ (x1 = {ii + 1} /\ y1 = 2))'
            '''}
        tester_safe |= {
            rf'''
            (x2 = {ii} /\ y2 = 2) => (
                   (x2 = {ii + 1} /\ y2 = 2)
                \/ (x2 = {ii} /\ y2 = 2)
                \/ (x2 = {ii + 1} /\ y2 = 1))'
            '''}
        tester_safe |= {
            rf'''
            (x2 = {ii} /\ y2 = 1) => (
                   (x2 = {ii + 1} /\ y2 = 1)
                \/ (x2 = {ii} /\ y2 = 1)
                \/ (x2 = {ii + 1} /\ y2 = 2))'
            '''}
        tester_safe |= {
            rf'''
            (x1 = {ii} /\ y1 = 2) => (
                   (x1 = {ii + 1} /\ y1 = 2)
                \/ (x1 = {ii} /\ y1 = 2)
                \/ (x1 = {ii + 1} /\ y1 = 1))'
            '''}
    # tester_safe |= {
    #     rf' ~ (x1 = {tracklength} /\ x2 = 0) '}
    tester_safe |= {
        rf'''
        (x2 = {tracklength} /\ y2 = 2)
        => (x2 = {tracklength} /\ y2 = 2)'
        '''}
    tester_safe |= {
        rf'''
        (x2 = {tracklength} /\ y2 = 1)
        => (x2 = {tracklength} /\ y2 = 1)'
        '''}
    tester_safe |= {
        r'''
        (x2 = 1 /\ y2 = 2) => (
               (x2 = 1 /\ y2 = 2)
            \/ (x2 = 2 /\ y2 = 2)
            \/ (x2 = 2 /\ y2 = 1))'
        '''}
    tester_safe |= {
        r'''
        (x2 = 1 /\ y2 = 1) => (
               (x2 = 1 /\ y2 = 1)
            \/ (x2 = 2 /\ y2 = 2)
            \/ (x2 = 2 /\ y2 = 1))'
        '''}
    tester_safe |= {
        rf'''
        (x1 = {tracklength} /\ y1 = 2)
        => (x1 = {tracklength} /\ y1 = 2)'
        '''}
    tester_safe |= {
        rf'''
        (x1 = {tracklength} /\ y1 = 1)
        => (x1 = {tracklength} /\ y1 = 1)'
        '''}
    tester_safe |= {
        rf'''
        (x1 = {tracklength - 1} /\ y1 = 1) => (
               (x1 = {tracklength - 1} /\ y1 = 1)
            \/ (x1 = {tracklength} /\ y1 = 2)
            \/ (x1 = {tracklength} /\ y1 = 1))'
        '''}
    tester_safe |= {
        rf'''
        (x1 = {tracklength - 1}
        /\ y1 = 2) => (
               (x1 = {tracklength - 1} /\ y1 = 2)
            \/ (x1 = {tracklength} /\ y1 = 1)
            \/ (x1 = {tracklength} /\ y1 = 2))'
        '''}

    # Synthesize specs
    ego_spec = Spec(
        sys_vars, sys_init,
        sys_safe, sys_prog)
    test_spec = Spec(
        tester_vars, tester_init,
        tester_safe, tester_prog)
    # st()
    return ego_spec, test_spec


def construct_sys_win_states(
        tracklength, states_in_winset):
    """System action states that are in tester winning set."""
    sys_states_in_winset = list()
    return sys_states_in_winset


def forward_step(
        tracklength, states_in_winset,
        x, y,
        x1, y1,
        x2, y2):
    sys_succ_in_winset = list()
    tester_succ = forward_step_tester(
        tracklength, x, y, x1, y1, x2, y2)
    for ti in tester_succ:
        system_succ = forward_step_sys(
            tracklength, x, y, x1, y1, x2, y2)
        flg = True
        for si in system_succ:
            state = dict(
                x=si[0], y=si[1], x1=ti[0],
                y1=ti[1], x2=ti[2], x3=ti[3])
            if state not in states_in_winset:
                flg = False
                break
        if flg:
            sys_state = dict(
                x=x, y=y, x1=x1,
                y1=y1, x2=x2, x3=y2)
            sys_succ_in_winset.append(sys_state)
    return sys_succ_in_winset


def forward_step_sys(
        tracklength,
        x, y,
        x1, y1,
        x2, y2):
    if x == tracklength:
        return [(x, y)]
    new_steps = [(x, y), (x + 1, 1), (x + 1, 2)]
    tester_states = [(x1, y1), (x2, y2)]
    for ti in tester_states:
        if ti in new_steps:
            new_steps.remove(ti)
    return new_steps


def forward_step_tester(
        tracklength,
        x, y,
        x1, y1,
        x2, y2):
    new_steps = [(x1, y1, x2, y2)]
    if x1 < tracklength:
        new_steps.append((x1 + 1, 1, x2, y2))
        new_steps.append((x1 + 1, 2, x2, y2))
    if x2 < tracklength - 1:
        new_steps.append((x1, y1, x2 + 1, 1))
        new_steps.append((x1, y1, x2 + 1, 2))
    if x1 < tracklength and x2 < tracklength - 1:
        new_steps.append((x1 + 1, 1, x2 + 1, 1))
        new_steps.append((x1 + 1, 2, x2 + 1, 2))
        new_steps.append((x1 + 1, 2, x2 + 1, 1))
        new_steps.append((x1 + 1, 1, x2 + 1, 2))
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

    If no controller exists,
    then raise an `Exception`.
    The returned controller is
    represented as a `networkx` graph.
    """
    z, yij, xijk = gr1.solve_streett_game(aut)
    gr1.make_streett_transducer(
        z, yij, xijk, aut)
    g = enum.action_to_steps(
        aut,
        env='env',
        sys='impl',
        qinit=aut.qinit)
    return g


def dump_graph_as_figure(g):
    """Create a PDF file showing the graph `g`."""
    h, _ = sym_enum._format_nx(g)
    pd = nx.drawing.nx_pydot.to_pydot(h)
    pd.write_pdf('game_states.pdf')


def check_all_states(
        tracklength, agentlist, w_set, aut):
    winning_set = w_set.find_winning_set(aut)
    num_test_agents = len(agentlist)
    states_in_winset = list()
    states_outside_winset = list()
    if num_test_agents == 1:
        for x in range(1, tracklength + 1):
            for y in range(1, 2 + 1):
                for x1 in range(1, tracklength + 1):
                    state = dict(x=x, y=y, x1=x1)
                    check_bdd = w_set.check_state_in_winset(
                        aut, winning_set, state)
                    print(state)
                    print(check_bdd)
    # `x2 < x1`, since `x2` is a second tester
    elif num_test_agents == 2:
        for x in range(1, tracklength + 1):
            for y in range(1, 2 + 1):
                for x1 in range(1, tracklength + 1):
                    for x2 in range(1, x1):
                        state = {
                            'x': x,
                            'y': y,
                            agentlist[0]: x1,
                            'y1': 2,
                            agentlist[1]: x2,
                            'y2': 2}
                        check_bdd = w_set.check_state_in_winset(
                            aut, winning_set, state)
                        if check_bdd:
                            states_in_winset.append(state)
                        else:
                            states_outside_winset.append(state)
                        print(state)
                        print(check_bdd)
    else:
        print('Too many agents')
    return states_in_winset, states_outside_winset


def _main():
    ex = 5  # Abstraction for the merge example
    # Constructing abstraction for
    # the merge example
    if ex == 6:
        tracklength = 3
        # spec_merge_in_front()
        # all_system(3)
        # spec_merge_in_front()
        # test_spec()
        # specs_for_entire_track(5)
        ego_spec, test_spec = specs_two_testers(tracklength)
        gr_spec = make_grspec(
            sys_spec=test_spec,
            env_spec=ego_spec)
        print(gr_spec.pretty())
        w_set = WinningSet()
        w_set.spec = gr_spec
        aut = w_set.make_compatible_automaton(gr_spec)
        # g = synthesize_some_controller(aut)
        # if g is None:
        #     print('Inconsistent specifications')
        # else:
        #     print('Controller found')
        agentlist = ['x1', 'x2']
        # pdb.set_trace()
        states_in_winset, states_out_winset = check_all_states(
            tracklength, agentlist, w_set, aut)
    # Constructing abstraction for
    # the merge example
    if ex == 5:
        tracklength = 4
        # spec_merge_in_front()
        # all_system(3)
        # spec_merge_in_front()
        # test_spec()
        # specs_for_entire_track(5)
        ego_spec, test_spec = specs_for_entire_track(tracklength)
        gr_spec = make_grspec(
            sys_spec=test_spec,
            env_spec=ego_spec)
        print(gr_spec.pretty())
        w_set = WinningSet()
        w_set.spec = gr_spec
        aut = w_set.make_compatible_automaton(gr_spec)
        # g = synthesize_some_controller(aut)
        # if g is None:
        #     print('Inconsistent specifications')
        # else:
        #     print('Controller found')
        agentlist = ['x1', 'x2']
        # pdb.set_trace()
        (states_in_winset, states_out_winset) = check_all_states(
                tracklength, agentlist, w_set, aut)
    winning_set = w_set.find_winning_set(aut)
    pdb.set_trace()
    #
    # - `(x, y)`: position of system under test
    # - `(x1, y1)`: position of leading tester car
    # - `(x2, y2)`: position of second tester car
    #
    # Domains of the position values can be found in
    # the variable declarations in the
    # `specs_for_entire_track()` function.
    #
    # To check if a state is in the winning set,
    # pass all values in dictionary form.
    # Each dictionary corresponds to one state.
    state = dict(
        x=1, y=1,
        x1=3, y1=2,
        x2=2, y2=2)
    # state = {'X0'}
    # `check_bdd` is a boolean.
    # `True` implies that `state` is in the winning set.
    check_bdd = w_set.check_state_in_winset(
        aut, winning_set, state)
    # pdb.set_trace()
    if check_bdd:
        print('State is in the winning set')
    else:
        print('State is not in the winning set')


if __name__ == '__main__':
    _main()
