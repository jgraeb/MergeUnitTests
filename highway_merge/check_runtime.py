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
from merge_receding_horizon_winsets import get_tester_states_in_winsets, specs_car_rh, get_winset_rh
from sim_merge import play_game
from test_parameters import TRACKLENGTH

def compute_winning_set_and_save_time(tracklength):
    merge_setting = "between"
    print('Synthesizing for {}'.format(tracklength))

    # compute here

    ego_spec, test_spec, Vij_dict, state_tracker, ver2st_dict, G, state_dict_test, state_dict_system = specs_car_rh(tracklength, merge_setting)
    Wij = dict()
    # Go through all the goal states
    # st()
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

    # Wij, Vij_dict, state_tracker, ver2st_dict = get_tester_states_in_winsets(tracklength, merge_setting)
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
    output_dir = os.getcwd()+'/saved_wsets/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'ws_out_file_'+str(tracklength)+'.p'
    filepath = output_dir + filename
    print('Saving winning set in pkl file')
    with open(filepath, 'wb') as pckl_file:
        pickle.dump(ws, pckl_file)

def plot_the_times(tracklength, times):
    # plt.style.use('_mpl-gallery')
    # st()
    # make data
    x = tracklength
    y = []
    for l in tracklength:
        y.append(times[l])
    # plot
    fig, ax = plt.subplots()

    plt.plot(x, y, linewidth=2.0)

    plt.xlabel('track length')
    plt.ylabel('time [s]')
    plt.title('Highway Merge: Runtime vs. Tracklength', fontsize = 15)

    plt.savefig('ws_runtime.png', dpi = 200, bbox_inches='tight')
    plt.savefig('ws_runtime.pdf', bbox_inches='tight')
    plt.show()

def save_times(tracklength,times):
    save_dict = dict()
    save_dict.update({'tracklength': tracklength})
    save_dict.update({'times': times})
    # save dict in pkl file
    output_dir = os.getcwd()+'/saved_data/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    filename = 'times_file.p'
    filepath = output_dir + filename
    print('Saving data in pkl file')
    with open(filepath, 'wb') as pckl_file:
        pickle.dump(save_dict, pckl_file)

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
    plt.title('Rollouts to Find Test Policy for Tracklength = '+str(TRACKLENGTH), fontsize = 15)
    # save the plots
    plt.savefig('ws_mcts_'+str(TRACKLENGTH)+'.png', dpi = 200, bbox_inches='tight')
    plt.savefig('ws_mcts_'+str(TRACKLENGTH)+'.pdf', bbox_inches='tight')
    plt.savefig('ws_mcts_'+str(TRACKLENGTH)+'.svg', bbox_inches='tight')
    plt.show()

def save_mcts_data(rollouts, data):
    save_dict = dict()
    save_dict.update({'tracklength': TRACKLENGTH})
    save_dict.update({'rollouts': rollouts})
    save_dict.update({'data': data})
    # save dict in pkl file
    output_dir = os.getcwd()+'/saved_data/'
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
            print("Synthesizing winning set for all goals takes {0}".format(t_all_goals))
            print('Tracklength: {0} took {1} s total and {2} for one single goal'.format(l,t,t2))
            times.update({l:t})

        print(times)
        save_times(tracklength,times)
        plot_the_times(tracklength, times)

if __name__=='__main__':
    check_MCTS = True
    check_WS = False
    if check_WS:
        print('Checking runtime for winning set synthesis')
        Lmax = 15 # Maximum tracklength
        tracklength = np.linspace(5, Lmax, Lmax-5+1)
        # tracklength = [5, 20]
        tracklength = [int(tli) for tli in tracklength]
        # tracklength=[18]
        times = dict()

        for l in tracklength:
            t, t2, t_all_goals = compute_winning_set_and_save_time(l)
            print("Synthesizing winning set for all goals takes {0}".format(t_all_goals))
            print('Tracklength: {0} took {1} s total and {2} for one single goal'.format(l,t,t2))
            times.update({l:t})
            # st()
        print(times)
        save_times(tracklength,times)
        plot_the_times(tracklength, times)

    if check_MCTS:
        Lmax = 15 # Maximum tracklength
        # tracklengths = np.linspace(5, Lmax, Lmax-5+1)
        rollouts_to_run = [1,2,3,4,5]#,6,7]#,8,9,10]#[1, 5,25]#,30,35,40,50,60,70,80,90,100]
        tracklengths = [15]
        data = dict()
        for l in tracklengths:
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
            print(cost_for_rollouts)
            data.update({l: cost_for_rollouts})
            print(data)
        # st()
        plot_mcts_data(data)
        save_mcts_data(tracklengths, rollouts_to_run, data)
