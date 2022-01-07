import sys
sys.path.append('..')
import numpy as np
from ipdb import set_trace as st
from collections import OrderedDict as od


def create_intersection_from_file(intersectionfile):
    map = od()
    f = open(intersectionfile, 'r')
    lines = f.readlines()
    len_y = len(lines)
    for i,line in enumerate(lines):
        for j,item in enumerate(line):
            if item != '\n':
                map[i,j] = item
    # make dictionary that maps each crosswalk state to a grid cell
    # currenly manual -> TODO crosswalk also automatically from file
    crosswalk = dict()
    start_cw = 2
    end_cw = 6
    y = 2
    for i, num in enumerate(range(2*start_cw,2*(end_cw+1))):
        crosswalk.update({i: (int(np.floor(num/2)), y)})
    return map, crosswalk

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

def check_all_states_in_fp(W, fixpt, aut, sys_st2ver_dict, test_st2ver_dict):
    # winning_set = w_set.find_winning_set(aut)
    print_flg = False
    states_in_winset = []
    states_outside_winset = []
    for state in sys_st2ver_dict.keys():
        check_bdd = w_set.check_state_in_fp(aut, winning_set, state)
        print(state)
        print(check_bdd)
        
    for state in test_st2ver_dict.keys():
        check_bdd = w_set.check_state_in_fp(aut, winning_set, state)
        print(state)
        print(check_bdd)
    return states_in_winset, states_outside_winset


## Check if states are in winning set for receding horizon winning sets:
# Filtering states in the winning set:
def check_all_states_in_winset_rh(W, fixpt, aut, state_test_dict, state_system_dict, goal_states, G, st2ver_dict, start_set):
    # winning_set = w_set.find_winning_set(aut)
    states_in_winset = []
    states_outside_winset = []
    # x2 < x1, since x2 is a second tester
    for state_node in start_set:
        state = ver2st_dict[state_node]
        check_bdd = w_set.check_state_in_fp(aut, winning_set, state)
        if check_bdd:
            state_node = get_dict_inv(ver2st_dict, state)
            check_flg = check_A_G_rh(state, state_node, tracklength, mode, state_test_dict, state_system_dict, goal_states, G)
            if check_flg:
                states_in_winset.append(state)
                if PRINT_STATES_IN_COMPUTATION:
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
