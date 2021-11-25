#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 18 13:20:15 2021

@author: apurvabadithela
"""

from correct_win_set import get_winset
import numpy as np
import matplotlib.pyplot as plt

tracklength = 4

def construct_win_sets(tracklength):
    set1 = "between"
    print("Winning sets for merge in between")
    states_in_winset_between = get_winset(tracklength, set1)
    print("--------------------------------- ")
    
    set2 = "front"
    print("Winning sets for merge in front")
    states_in_winset_front = get_winset(tracklength, set2)
    print("--------------------------------- ")
    
    set3 = "back"
    print("Winning sets for merge in back")
    states_in_winset_back = get_winset(tracklength, set3)
    print("--------------------------------- ")
    return states_in_winset_between, states_in_winset_front, states_in_winset_back

# Checking if a state is in a merge in between win set:
def composed_test_winset_query(state, states_in_winset_between):
    if state in states_in_winset_between:
        return True
    else:
        return False


# Checking if a state is in a merge in front win set:
def front_merge_winset_query(state, states_in_winset_front):
    if state in states_in_winset_front:
        return True
    else:
        return False
    
    
# Checking if a state is in a merge in back win set:
def back_merge_winset_query(state, states_in_winset_back):
    if state in states_in_winset_back:
        return True
    else:
        return False
    
# Check if all of the merge in between winning set states are in between the two
# Computing winning set percentage:
def find_intersection(s1, s2):
    intersection = []
    for s in s1:
        if s in s2 and s not in intersection:
            intersection.append(s)
    for s in s2:
        if s in s1 and s not in intersection:
            intersection.append(s)
    return intersection

# Check if intersection equals merge in between winset:
def int_merge_in_between(intersection, states_in_winset_between):
    for s in intersection:
        if s not in states_in_winset_between:
            print("State not in merge in between winset: ")
            print(s)
    for s in states_in_winset_between:
        if s not in intersection:
            print("State not in the intersection: ")
            print(s)
states_in_winset_between, states_in_winset_front, states_in_winset_back = construct_win_sets(tracklength)
     
intersection = find_intersection(states_in_winset_front, states_in_winset_back)
print("Number of states in intersection: ")
print(len(intersection))

int_merge_in_between(intersection, states_in_winset_between)
print("Number of states in front_winset: ")
print(len(states_in_winset_front))

print("Number of states in back_winset: ")
print(len(states_in_winset_back))

# int_card = []
# front_card = []
# back_card = []
# track_length = []
# for tl in range(3, 11):
#     track_length.append(tl)
#     states_in_winset_between, states_in_winset_front, states_in_winset_back = construct_win_sets(tl)
#     intersection = find_intersection(states_in_winset_front, states_in_winset_back)
#     int_card.append(len(intersection))
#     front_cardinal = len(states_in_winset_front)
#     back_cardinal = len(states_in_winset_back)
#     front_card.append(front_cardinal)
#     back_card.append(back_cardinal)

# int_card = np.array(int_card)
# front_card = np.array(front_card)
# back_card = np.array(back_card)
# track = np.array(track_length)
# fig, ax = plt.subplots()
# plt.plot(track, int_card/front_card, 'rs', track, int_card/back_card, 'bs')
# plt.savefig("ratio_lines.png")

# Proposition coverage:
def propositions_rel_states(state):
    # - - S
    # - T T
    p1 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x1==x and x2==x1-1)
    # - S -
    # - T T
    p2 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x1==x+1 and x2==x)
    # - - -
    # T S T
    p3 = lambda x, y, x1, y1, x2, y2: (y==2 and y1==2 and y2==2 and x2==x-1 and x1==x+1)
    # merge behind
    # - - -
    # - S T
    p4 = lambda x, y, x1, y1, x2, y2: (y==2 and y1==2 and y2==2 and x1==x+1 and x2!=x1-2)
    p5 = lambda x, y, x1, y1, x2, y2: (y==2 and y1==2 and y2==2 and x2==x+1)
    # merge in front:
    p6 = lambda x, y, x1, y1, x2, y2: (y==2 and y1==2 and y2==2 and x1==x-1)
    p7 = lambda x, y, x1, y1, x2, y2: (y==2 and y1==2 and y2==2 and x2==x-1 and x1!=x+1)
    p8 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2==x-1 and x1==x+1)
    p9 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2==x+1 and x1==x+3)
    p10 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2==x+1 and x1==x+2)
    p11 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2==x and x1==x+2)
    p12 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2==x+1 and x1!=x+2)
    p13 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2==x and x1!=x+2)
    p14 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2!=x-1 and x1==x+1)
    p15 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2!=x-1 and x1==x)
    p16 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2==x+2)
    p17 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2==x-1 and x1!=x+1)
    p18 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and x2!=x-1 and x1==x+2)

    props = [p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18]
    for ii in range(len(props)):
        pi = props[ii]
        x = state['x']
        y = state['y']
        x1 = state['x1']
        y1 = state['y1']
        x2 = state['x2']
        y2 = state['y2']
        if pi(x,y,x1,y1,x2,y2):
            return ii
    return "None"

# Propositions on the other states:
def propositions_simple(state):
    p1 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and (x2==x-1 or x1==x-1))
    p2 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and (x2==x+1 or x1==x+1))
    p3 = lambda x, y, x1, y1, x2, y2: (y==1 and y1==2 and y2==2 and (x2==x or x1==x))
    p4 = lambda x, y, x1, y1, x2, y2: (y==2 and y1==2 and y2==2 and (x2==x-1 or x1==x+1))
    p5 = lambda x, y, x1, y1, x2, y2: (y==2 and y1==2 and y2==2 and (x2==x+1 or (x1==x+1 and x2!=x-1)))
    p6 = lambda x, y, x1, y1, x2, y2: (y==2 and y1==2 and y2==2 and ((x2==x-1 and x1!=x+1) or x1==x-1)) 
    props = [p1, p2, p3, p4, p5, p6]
    
    for ii in range(len(props)):
        pi = props[ii]
        x = state['x']
        y = state['y']
        x1 = state['x1']
        y1 = state['y1']
        x2 = state['x2']
        y2 = state['y2']
        if pi(x,y,x1,y1,x2,y2):
            return ii
    return "None"

def compute_num_props(winset):
    props_idx = []
    states_uncovered = []
    for wi in winset:
        idx = propositions_simple(wi)
        if idx!="None" and idx not in props_idx:
            props_idx.append(idx)
        if idx == "None":
            states_uncovered.append(wi)
    return props_idx, states_uncovered


int_card = []
front_card = []
back_card = []
track_length = []
for tl in range(3, 11):
    track_length.append(tl)
    states_in_winset_between, states_in_winset_front, states_in_winset_back = construct_win_sets(tl)
    intersection = find_intersection(states_in_winset_front, states_in_winset_back)
    inter, int_unc = compute_num_props(states_in_winset_between) 
    front, front_unc = compute_num_props(states_in_winset_front) 
    back, back_unc = compute_num_props(states_in_winset_back)
    int_cardinal = len(inter)
    front_cardinal = len(front)
    back_cardinal = len(back)
    int_card.append(int_cardinal)
    front_card.append(front_cardinal)
    back_card.append(back_cardinal)

int_card = np.array(int_card)
front_card = np.array(front_card)
back_card = np.array(back_card)
track = np.array(track_length)
fig, ax = plt.subplots()
plt.plot(track, int_card/front_card, 'rs', track, int_card/back_card, 'bs')
plt.savefig("prop_simple.png")