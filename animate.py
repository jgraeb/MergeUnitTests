import copy as cp
import sys
from ipdb import set_trace as st
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import glob
#from rose import Car, Map, CAR_COLORS
from PIL import Image
import _pickle as pickle
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator,
        FormatStrFormatter, AutoMinorLocator)
from matplotlib.collections import PatchCollection
import imageio

TILESIZE = 20
CAR_COLORS = ['blue', 'red']

main_dir = os.path.dirname(os.path.dirname(os.path.realpath("__file__")))
car_figs = dict()
for color in CAR_COLORS:
    car_figs[color] = main_dir + '/CompositionalTesting/imglib/' + color + '_car.png'


def draw_map(map):
    lanes = map.lanes
    width = map.width
    x_min = 0
    x_max = width * TILESIZE
    y_min = 0
    y_max = lanes * TILESIZE
    #x_min, x_max, y_min, y_max = get_map_corners(map)
    ax.axis('equal')
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    # fill in the road regions
    road_tiles = []
    width_tiles = np.arange(0,width+1)*TILESIZE
    lanes_tiles = np.arange(0,lanes+1)*TILESIZE

    for i in np.arange(lanes):
        for k in np.arange(width):
            tile = patches.Rectangle((width_tiles[k],lanes_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='k', alpha=0.4)
            road_tiles.append(tile)
    ax.add_collection(PatchCollection(road_tiles, match_original=True))

    plt.gca().invert_yaxis()

def draw_car(car_data):
    if car_data[0]=='ego':
        color = 'red'
    else:
        color = 'blue'
    theta_d = 0
    name, x_tile, y_tile = car_data
    x = (x_tile-1) * TILESIZE
    y = (y_tile-1) * TILESIZE
    car_fig = Image.open(car_figs[color])
    car_fig = car_fig.rotate(theta_d, expand=False)
    offset = 0.1
    ax.imshow(car_fig, zorder=1, interpolation='none', extent=[x+2, x+TILESIZE-2, y+2, y+TILESIZE-2])

def plot_ego_cars(agents):
    for i, agent in enumerate(agents):
        draw_car(agent)

def plot_env_cars(agents):
    for i, agent in enumerate(agents):
        draw_car(agent)

def animate_images(output_dir):
    # Create the frames
    frames = []
    imgs = glob.glob(output_dir+'plot_'"*.png")
    imgs.sort()
    for i in imgs:
        new_frame = Image.open(i)
        frames.append(new_frame)

    # Save into a GIF file that loops forever
    frames[0].save(output_dir + 'png_to_gif.gif', format='GIF',
            append_images=frames[1:],
            save_all=True,
            duration=200, loop=3)

def traces_to_animation(filename, output_dir):
    # extract out traces from pickle file
    with open(filename, 'rb') as pckl_file:
        traces = pickle.load(pckl_file)
    ##
    t_start = 0
    t_end = len(traces)
    # t_start = traces[0].timestamp
    # t_end = traces[-1].timestamp
    map = traces[0].map
    #import pdb; pdb.set_trace()
    global ax
    fig, ax = plt.subplots()

    t_array = np.arange(t_end)
    # plot map once
    for t in t_array:
        print(t)
        plt.gca().cla()
        draw_map(map)
        ego_agents = traces[t].ego
        env_agents = traces[t].env
        plot_ego_cars(ego_agents)
        plot_env_cars(env_agents)
        plot_name = str(t).zfill(5)
        img_name = output_dir+'/plot_'+plot_name+'.png'
        fig.savefig(img_name)
    animate_images(output_dir)

def make_animation():
    output_dir = os.getcwd()+'/animations/gifs/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    traces_file = os.getcwd()+'/saved_traces/sim_trace.p'
    traces_to_animation(traces_file, output_dir)

if __name__ == '__main__':
    make_animation()
