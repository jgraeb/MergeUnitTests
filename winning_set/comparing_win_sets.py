#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 18 13:20:15 2021

@author: apurvabadithela
"""

from correct_win_set import get_winset
import numpy as np


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

states_in_winset_between, states_in_winset_front, states_in_winset_back = construct_win_sets(tracklength)

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