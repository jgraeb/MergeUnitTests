import numpy as np
import matplotlib.pyplot as plt
import time
import os
import pickle
from ipdb import set_trace as st
from merge_receding_horizon_winsets import get_tester_states_in_winsets, specs_car_rh, get_winset_rh

def compute_winning_set_and_save_time(tracklength):
    merge_setting = "between"
    print('Synthesizing for {}'.format(tracklength))

    # compute here

    ego_spec, test_spec, Vij_dict, state_tracker, ver2st_dict, G, state_dict_test, state_dict_system = specs_car_rh(tracklength, merge_setting)
    Wij = dict()
    # Go through all the goal states
    # st()
    tic = time.time()
    for i, key in enumerate(Vij_dict.keys()):
        tic2 = time.time()
        Wj = get_winset_rh(tracklength, merge_setting, Vij_dict[key], state_tracker, ver2st_dict,ego_spec, test_spec, state_dict_test, state_dict_system, G)
        toc2 = time.time()
        delta_t2 = toc2 - tic2
        print('Goal {0} takes {1} s'.format(i,delta_t2))
        Wij.update({key: Wj})

    # Wij, Vij_dict, state_tracker, ver2st_dict = get_tester_states_in_winsets(tracklength, merge_setting)
    toc = time.time()
    delta_t = toc - tic
    save_winning_set(tracklength, Wij, Vij_dict, state_tracker, ver2st_dict)
    return delta_t, delta_t2

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
    plt.style.use('_mpl-gallery')
    # st()
    # make data
    x = tracklength
    y = []
    for l in tracklength:
        y.append(times[l])

    # plot
    fig, ax = plt.subplots()

    plt.plot(x, y, linewidth=2.0)

    # ax.set(xlim=(0, 8), xticks=np.arange(1, 8),
    #        ylim=(0, 8), yticks=np.arange(1, 8))

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


if __name__=='__main__':
    print('Checking runtime for winning set synthesis')
    tracklength = [5,6,7,8,9,10,12,14,16,18,20]
    times = dict()

    for l in tracklength:
        t, t2 = compute_winning_set_and_save_time(l)
        print('Tracklength: {0} took {1} s total and {2} for one single goal'.format(l,t,t2))
        times.update({l:t})
    print(times)
    save_times(tracklength,times)
    plot_the_times(tracklength, times)
