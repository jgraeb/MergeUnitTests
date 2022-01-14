import numpy as np
import matplotlib.pyplot as plt
import time
import os
import math
import pickle
from ipdb import set_trace as st
import sys
# import seaborn as sns
sys.path.append('..')
from highway_merge.merge_receding_horizon_winsets import get_tester_states_in_winsets, specs_car_rh, get_winset_rh, specs_full
from sim_merge import play_game
from highway_merge.test_parameters import TRACKLENGTH
from winning_set.correct_win_set import specs_for_entire_track, make_grspec, Spec, WinningSet, check_all_states_in_winset, specs_car_merge, specs_car_merge_for_goal


def compute_winning_set_and_save_time(tracklength):
    merge_setting = "between"
    print('Synthesizing for {}'.format(tracklength))

    ego_spec, test_spec, Vij_dict, state_tracker, ver2st_dict, G, state_dict_test, state_dict_system = specs_car_rh(tracklength, merge_setting)
    Wij = dict()

    t_winset_all_goals = 0
    tic = time.time()
    for i, key in enumerate(Vij_dict.keys()):
        tic2 = time.time()
        Wj, t_goal = get_winset_rh(tracklength, merge_setting, Vij_dict[key], state_tracker, ver2st_dict,ego_spec, test_spec, state_dict_test, state_dict_system, G)
        toc2 = time.time()
        delta_t2 = toc2 - tic2
        print('Goal {0} takes {1} s'.format(i,delta_t2))
        Wij.update({key: Wj})
        t_winset_all_goals += t_goal

    toc = time.time()
    delta_t = toc - tic
    save_winning_set(tracklength, Wij, Vij_dict, state_tracker, ver2st_dict)
    return delta_t, delta_t2, t_winset_all_goals

def save_winning_set(tracklength, Wij, Vij_dict, state_tracker, ver2st_dict):
    # save objects in dictionary
    ws = dict()
    ws.update({'Wij': Wij})
    ws.update({'Vij_dict': Vij_dict})
    ws.update({'state_tracker': state_tracker})
    ws.update({'ver2st_dict': ver2st_dict})
    # save dict in pkl file
    output_dir = os.getcwd()+'/highway_merge/saved_filters/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'ws_out_file_'+str(tracklength)+'.p'
    filepath = output_dir + filename
    print('Saving winning set in pkl file')
    with open(filepath, 'wb') as pckl_file:
        pickle.dump(ws, pckl_file)

def plot_the_times(tracklength, times, times_orig):
    # make data
    x = tracklength
    y = []
    for l in tracklength:
        y.append(times[l])
    y2 = times_orig
    # plot
    fig, ax = plt.subplots()
    plt.plot(x, y, linewidth=2.0, label='Receding Horizon')
    plt.plot(x[:len(y2)], y2, linewidth=2.0, label='Full Horizon')

    new_list = range(math.floor(min(x)), math.ceil(max(x))+1)
    plt.xticks(new_list)

    plt.xlabel('Track Length')
    plt.ylabel('Time [s]')
    plt.title('Test Policy Filter Synthesis Runtime', fontsize = 15)
    plt.legend()
    output_dir = os.getcwd()+'/highway_merge/runtime/'

    plt.savefig(output_dir + 'ws_runtime.png', dpi = 200, bbox_inches='tight')
    plt.savefig(output_dir + 'ws_runtime.pdf', bbox_inches='tight')

    plt.show()

def save_times(tracklength,times,times_orig):
    save_dict = dict()
    save_dict.update({'tracklength': tracklength})
    save_dict.update({'times': times})
    save_dict.update({'times_orig': times_orig})
    # save dict in pkl file
    output_dir = os.getcwd()+'/highway_merge/runtime/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'ws_synthsis_times_file.p'
    filepath = output_dir + filename
    print('Saving data in pkl file')
    with open(filepath, 'wb') as pckl_file:
        pickle.dump(save_dict, pckl_file)

def original_specs(Lmax):
    tracklength = np.linspace(5, Lmax, Lmax-5+1)
    tracklength = [int(tli) for tli in tracklength]
    times = []
    merge_setting = "between"
    for l in tracklength:
        try:
            ego_spec, test_spec = specs_car_merge(l)
            gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
            # print(gr_spec.pretty())
            w_set = WinningSet()
            tic = time.time()
            w_set.set_spec(gr_spec)
            aut = w_set.make_compatible_automaton(gr_spec)
            # g = synthesize_some_controller(aut)
            agentlist = ['x1', 'x2']
            fp = w_set.find_winning_set(aut)
            # print("Printing states in fixpoint: ")
            # states_in_fp, states_out_fp = check_all_states_in_fp(tracklength, agentlist, w_set, fp, aut)
            # print(" ")
            # print("Printing states in winning set: ")
            mode="between"
            states_in_winset = check_all_states_in_winset(l, agentlist, w_set, fp, aut, mode)
            toc = time.time()
            del_t = toc-tic
            times.append(del_t)
        except:
            del_t = None
        print('Tracklength: {0}, Time: {1}'.format(l,del_t))
    return tracklength, times

def original_specs_for_each_goal_separately(Lmax):
    tracklength = np.linspace(5, Lmax, Lmax-5+1)
    tracklength = [int(tli) for tli in tracklength]
    times = []
    merge_setting = "between"
    for l in tracklength:
        time_for_l = 0
        for goal_pos in range(2,l):
            try:
            # if True:
                # st()
                ego_spec, test_spec = specs_car_merge_for_goal(l, goal_pos)
                gr_spec = make_grspec(test_spec, ego_spec) # Placing test_spec as sys_spec and sys_spec as env_spec to
                # print(gr_spec.pretty())
                w_set = WinningSet()
                tic = time.time()
                w_set.set_spec(gr_spec)
                aut = w_set.make_compatible_automaton(gr_spec)
                # g = synthesize_some_controller(aut)
                agentlist = ['x1', 'x2']
                fp = w_set.find_winning_set(aut)
                # print("Printing states in fixpoint: ")
                # states_in_fp, states_out_fp = check_all_states_in_fp(tracklength, agentlist, w_set, fp, aut)
                # print(" ")
                # print("Printing states in winning set: ")
                mode="between"
                states_in_winset = check_all_states_in_winset(l, agentlist, w_set, fp, aut, mode)
                toc = time.time()
                del_t = toc-tic
                print('Tracklength {0} - Goal {1}, Time: {2}'.format(l,goal_pos,del_t))
            except:
                del_t = None
                time_for_l = None
            if del_t:
                time_for_l = time_for_l + del_t

        print('Tracklength: {0}, Time: {1}'.format(l,time_for_l))
        times.append(time_for_l)
    return tracklength, times

def plot_mcts_data(data):
    # get the data
    x = list(data[TRACKLENGTH].keys())
    y = np.array([(np.mean(data[TRACKLENGTH][k]))/(TRACKLENGTH-1) for k in x])
    error = np.array([np.std(data[TRACKLENGTH][k])/(TRACKLENGTH-1) for k in x])
    error_min = np.array([np.min(data[TRACKLENGTH][k])/(TRACKLENGTH-1) for k in x])
    error_max = np.array([np.max(data[TRACKLENGTH][k])/(TRACKLENGTH-1) for k in x])

    plt.plot(x, y, color= '#424b7f')
    plt.fill_between(x, y-(y-error_min), y+(error_max-y), alpha=0.2, edgecolor='#0c8ecb', facecolor='#9bdbe4', linewidth=4, linestyle='dotted')
    plt.fill_between(x, y-error, y+error, alpha=0.6, edgecolor='#0c8ecb', facecolor='#9bdbe4', linewidth=4, linestyle='dotted')

    new_list = range(math.floor(min(x)), math.ceil(max(x))+1)
    plt.xticks(new_list)
    plt.xlabel('Number of Rollouts')
    plt.ylabel('Terminal Cost')
    plt.title('Rollouts to Find Test Policy for Track Length = '+str(TRACKLENGTH), fontsize = 15)
    # save the plots
    output_dir = os.getcwd()+'/highway_merge/runtime/'

    plt.savefig(output_dir + 'mcts_'+str(TRACKLENGTH)+'.png', dpi = 200, bbox_inches='tight')
    plt.savefig(output_dir + 'mcts_'+str(TRACKLENGTH)+'.pdf', bbox_inches='tight')
    # plt.savefig(output_dir + 'mcts_'+str(TRACKLENGTH)+'.svg', bbox_inches='tight')
    plt.show()

def save_mcts_data(rollouts, data):
    save_dict = dict()
    save_dict.update({'tracklength': TRACKLENGTH})
    save_dict.update({'rollouts': rollouts})
    save_dict.update({'data': data})
    # save dict in pkl file
    output_dir = os.getcwd()+'/highway_merge/runtime/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'mcts_comparison_file_'+str(TRACKLENGTH)+'.p'
    filepath = output_dir + filename
    print('Saving data in pkl file')
    with open(filepath, 'wb') as pckl_file:
        pickle.dump(save_dict, pckl_file)

def run_mcts_convergence_check(rollouts_to_run):
    rollouts_to_run = [1,2,3,4,5]

    data = dict()
    cost_for_rollouts = dict()
    for num_rollouts in rollouts_to_run:
        cost = []
        for i in range(0,50):
            ego_trace, env_trace, game_trace = play_game(num_rollouts)
            ego_cost = ego_trace['x'][-1]
            cost.append(ego_cost)
        cost_for_rollouts.update({num_rollouts: cost})
    data.update({TRACKLENGTH: cost_for_rollouts})
    plot_mcts_data(data)
    save_mcts_data(rollouts_to_run, data)

def run_winning_set_filter_synthesis_runtime_check(Lmax):
        print('Checking runtime for winning set synthesis')
        # minimum tracklength 5
        tracklength = np.linspace(5, Lmax, Lmax-5+1)
        tracklength = [int(tli) for tli in tracklength]
        times = dict()
        for l in tracklength:
            t, t2, t_all_goals = compute_winning_set_and_save_time(l)
            # print("Synthesizing winning set for all goals takes {0}".format(t_all_goals))
            # print('Tracklength: {0} took {1} s total and {2} for one single goal'.format(l,t,t2))
            times.update({l:t})

        print(tracklength)
        print(times)
        # tracklengths, times_orig = original_specs(Lmax)
        tracklengths, times_all_goals = original_specs_for_each_goal_separately(Lmax)
        # print(times)
        # print(times_orig)
        # st()
        save_times(tracklengths,times, times_all_goals)
        plot_the_times(tracklength, times, times_all_goals)


if __name__=='__main__':
    check_MCTS = True
    check_WS = False
    if check_WS:
        print('Checking runtime for winning set synthesis')
        Lmax = 15 # Maximum tracklength
        tracklength = np.linspace(5, Lmax, Lmax-5+1)
        tracklength = [int(tli) for tli in tracklength]
        times = dict()

        for l in tracklength:
            t, t2, t_all_goals = compute_winning_set_and_save_time(l)
            print("Synthesizing winning set for all goals takes {0}".format(t_all_goals))
            print('Tracklength: {0} took {1} s total and {2} for one single goal'.format(l,t,t2))
            times.update({l:t})

        print(times)
        save_times(tracklength,times)
        plot_the_times(tracklength, times)

    if check_MCTS:
        Lmax = 15 # Maximum tracklength
        rollouts_to_run = [1,2,3,4,5]
        data = dict()
        cost_for_rollouts = dict()
        for num_rollouts in rollouts_to_run:
            cost = []
            for i in range(0,50):
                ego_trace, env_trace, game_trace = play_game(num_rollouts)
                # st()
                ego_cost = ego_trace['x'][-1]
                print(ego_cost)
                cost.append(ego_cost)
            cost_for_rollouts.update({num_rollouts: cost})
        data.update({TRACKLENGTH: cost_for_rollouts})
        print(data)

        plot_mcts_data(data)
        save_mcts_data(rollouts_to_run, data)
