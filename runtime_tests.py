'''
Set `CHECK_MCTS` to True for the MCTS runtime check, or set `CHECK_WS` for the
receding horizon winning set synthesis runtime check.

MCTS runtime check:
- Set the desired tracklength in highway_merge/test_parameters.py

Winning set runtime check:
- Set the desired maximum track length in `Lmax` below, the minimum is set to 5.
'''
import numpy as np
import matplotlib.pyplot as plt
import time
import os
import math
import pickle
from ipdb import set_trace as st
import sys
sys.path.append('..')
from merge_receding_horizon_winsets import get_tester_states_in_winsets, specs_car_rh, get_winset_rh
from sim_merge import play_game
from test_parameters import TRACKLENGTH
from highway_merge.check_runtime import run_mcts_convergence_check

CHECK_MCTS = True
CHECK_WS = False

if __name__ == '__main__':

    if CHECK_MCTS:
        # Set desired tracklength in highway_merge/test_parameters.py
        rollouts_to_run = [1,2,3,4,5]
        rollouts_to_run = np.linspace(1, 5, 5)
        run_mcts_convergence_check(rollouts_to_run)

    elif CHECK_WS:
        max_tracklength = 10
        un_winning_set_filter_synthesis_runtime_check(max_tracklength)
