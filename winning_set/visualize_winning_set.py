"""
Winning Set visualization for Compositional Testing, applied to the merge example.
author: Josefine Graebener
Caltech, Oct 18 2021
"""

from ipdb import set_trace as st
import numpy as np
import matplotlib.pyplot as plt
from comparing_win_sets import *
from copy import deepcopy


"""
Set these flags to visualize the winning set:

shade_duplicate_states - States in the winning set are colored light blue if they were already visited.
draw_states_not_in_ws - States not in the winning set are omitted.
show_example_trace - Draw a trace through the winning set, specify trace in STATE_TRACE.

s_init - Initial state for winning set computation and centered in the visualization.
STATE_TRACE - List of states that is drawn on the polar lot.

"""
shade_duplicate_states = False
show_example_trace = True
draw_states_not_in_ws = False
track_length = 4
show_children_of_non_ws_states = True


def find_children_states(state):
    """
    Find all the children of a given state.
    """
    children = []
    if state['turn'] == 't':
        actions = {'stay': (0,0), 'move': (1,0)}
        for act2 in actions:
            x2 = state['t2'][0]+actions[act2][0]
            y2 = state['t2'][1]+actions[act2][1]
            if x2 > track_length:
                continue
            else:
                for act1 in actions:
                    x1 = state['t1'][0]+actions[act1][0]
                    y1 = state['t1'][1]+actions[act1][1]
                    if (x1,y1) != (x2,y2) and (x2,y2)!= state['sys'] and (x1,y1) != state['sys']:
                        children.append({'sys': state['sys'], 't1': (x1,y1), 't2': (x2,y2), 'turn': 's'})
    else:
        actions = {'stay': (0,0), 'move': (1,0), 'merge': (1,1)}
        for act in actions:
            x = state['sys'][0]+actions[act][0]
            y = state['sys'][1]+actions[act][1]
            if x > track_length:
                continue
            else:
                if (x,y) != state['t1'] and (x,y) != state['t2']:
                    children.append({'sys': (x,y), 't1': state['t1'], 't2': state['t2'], 'turn': 't'})
    return children

def find_next_generation(states,trace, states_in_winset):
    """
    Find all states in the next step, given the states in the previous step.
    """
    children = []
    for i,state in enumerate(states):
        if not check_final(state):# and composed_test_winset_query(ws_query_form(state), states_in_winset):
            state_children = find_children_states(state)
            if i == trace[-1]:
                # st()
                if len(STATE_TRACE) > len(trace):
                    if STATE_TRACE[len(trace)] in state_children:
                        # st()
                        dx = state_children.index(STATE_TRACE[len(trace)])
                        number = len(children) + dx
                        trace.append(number)
            children = children + state_children
    return children, trace

def duplicate(state, all_states):
    """
    Check if a state has already appeared in the same or a previous step.
    """
    if state in all_states:
        return True
    else:
        return False

def spec_check(state):
    """
    Return True if the state passes the safety filter.
    """
    if state['t1'][0] == state['sys'][0] + 1 and state['t1'][1] == state['sys'][1] + 1:
        return True
    elif state['t2'][0] == state['sys'][0] + 1 and state['t2'][1] == state['sys'][1] + 1:
        return True
    elif state['t2'][0] == state['sys'][0] + 2 and state['t1'][0] == state['sys'][0]:
        return True
    elif state['t2'][0] == state['sys'][0] + 1 and state['t1'][0] == state['sys'][0] -1 and state['sys'][1] == 2:
        return True
    else:
        return False

def check_final(state):
    """
    Return True if this is a final state in the simulation - no further rollouts from there.
    """
    # if state['t2'][0] == state['sys'][0] + 1 and state['t1'][0] == state['sys'][0] -1 and state['sys'][1] == 2:
    if state['sys'][1] == 2:
        return True
    else:
        return False

def in_winning_set(state):
    """
    Check if the state is in the winning set.
    """
    if state['turn'] == 's':
        return spec_check(state)
    else:
        children = find_children_states(state)
        for child in children:
            if spec_check(child):
                return True
        return False

def ws_query_form(state):
    """
    Reformat the state such that it can be passed into the winning set query function.
    """
    return {'x': state['sys'][0], 'y': state['sys'][1], 'x1': state['t2'][0], 'y1': state['t2'][1], 'x2': state['t1'][0], 'y2': state['t1'][1]}

def visualize_ws(states_in_winset, num_generations, s_init):
    """
    Draw the polar plot to visualize the winning set from an initial condition.
    """
    # Figure properties
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_axisbelow(True)
    ax.xaxis.set_visible(False)
    # ax.set_rscale('symlog')

    trace = [0]
    all_states = []
    # s_init = {'sys': (1,1), 't1': (1,2), 't2': (2,2), 'turn': 't'}
    all_states = all_states + [s_init]
    theta = 0
    r = 0
    c = ax.scatter(theta, r, c='b', s=22, cmap='hsv', alpha=0.75, zorder = 2)
    states = [s_init]
    # num_generations = 7
    data = []
    data.append([s_init])
    for gen in range(1,num_generations):
        children, trace = find_next_generation(states, trace,states_in_winset)
        num_children = len(children)
        data.append(deepcopy(children))
        for i,child in enumerate(children):
            if composed_test_winset_query(ws_query_form(child), states_in_winset):
                if check_final(child):
                    color = 'deeppink'
                elif duplicate(child, all_states) and shade_duplicate_states:
                    color = 'cornflowerblue'
                else:
                    all_states = all_states + [child]
                    color = 'b'
            else:
                color = 'silver'
            if not draw_states_not_in_ws and color == 'silver' and gen in [4,5,6]:
                    continue
            else:
                theta = 2 * np.pi * i / num_children
                offset = 2 * np.pi / num_children
                r = 10*gen
                area = 1/gen*22
                c = ax.scatter(theta, r, c=color, s=area, cmap='hsv', alpha=0.75, zorder = 2)
            states = children
    if show_example_trace:
        for i in range(0,len(trace)-1):
            ax.plot((2 * np.pi * trace[i] / len(data[i]), 2 * np.pi * trace[i+1] / len(data[i+1])), (10*i, 10*(i+1)), c = 'silver', zorder = 1)

    # print(trace)
    # plt.show()

def print_receding_ws(states_in_winset_between, STATE_TRACE):
    num_generations = 3
    horizons = int(np.ceil((len(STATE_TRACE)-1)/2))
    # st()
    for plots in range(0,horizons):
        num_generations = 3
        visualize_ws(states_in_winset_between, num_generations, STATE_TRACE[plots*num_generations-1])
    pass

STATE_TRACE = [{'sys': (1,1), 't1': (1,2), 't2': (2,2), 'turn': 't'},
               {'sys': (1,1), 't1': (2,2), 't2': (3,2), 'turn': 's'},
               {'sys': (2,1), 't1': (2,2), 't2': (3,2), 'turn': 't'},
               {'sys': (2,1), 't1': (2,2), 't2': (4,2), 'turn': 's'},
               {'sys': (3,2), 't1': (2,2), 't2': (4,2), 'turn': 't'}]

if __name__ == '__main__':
    states_in_winset_between, states_in_winset_front, states_in_winset_back = construct_win_sets(tracklength)
    s_init = {'sys': (1,1), 't1': (1,2), 't2': (2,2), 'turn': 't'}
    # s_init = {'sys': (2,1), 't1': (2,2), 't2': (3,2), 'turn': 't'}
    STATE_TRACE = [{'sys': (1,1), 't1': (1,2), 't2': (2,2), 'turn': 't'},
                   {'sys': (1,1), 't1': (2,2), 't2': (3,2), 'turn': 's'},
                   {'sys': (2,1), 't1': (2,2), 't2': (3,2), 'turn': 't'},
                   {'sys': (2,1), 't1': (2,2), 't2': (4,2), 'turn': 's'},
                   {'sys': (3,2), 't1': (2,2), 't2': (4,2), 'turn': 't'}]
    num_generations = 7
    visualize_ws(states_in_winset_back, num_generations, s_init)

    # print_receding_ws(states_in_winset_between, STATE_TRACE)
    plt.show()
