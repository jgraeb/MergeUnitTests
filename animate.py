import copy as cp
import sys
from ipdb import set_trace as st
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import glob
from PIL import Image
import _pickle as pickle
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator,
        FormatStrFormatter, AutoMinorLocator)
from matplotlib.collections import PatchCollection
import imageio

TILESIZE = 50
CAR_COLORS = ['blue', 'red']
LIGHT_COLORS = ['green', 'red', 'yellow']
ORIENTATIONS = {'n': 270, 'e': 0, 's': 90,'w':180, 'ne':315, 'nw':225, 'se':45, 'sw':135}
START_CROSSWALK = 1
END_CROSSWALK = 5
CROSSWALK_V = 2
CROSSWALK_LOCATIONS = dict()
for i, num in enumerate(range(2*START_CROSSWALK,2*(END_CROSSWALK+1))):
    CROSSWALK_LOCATIONS.update({i: (num/2, CROSSWALK_V)})
# st()

main_dir = os.path.dirname(os.path.dirname(os.path.realpath("__file__")))
car_figs = dict()
for color in CAR_COLORS:
    car_figs[color] = main_dir + '/CompositionalTesting/imglib/' + color + '_car.png'
ped_figure = main_dir + '/CompositionalTesting/imglib/pedestrian_img.png'

light_figs = dict()
for color in LIGHT_COLORS:
    light_figs[color] = main_dir + '/CompositionalTesting/imglib/' + color + '_light.png'

def draw_map(map, merge = False):
    if merge:
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
        ax.xaxis.set_visible(False)
        ax.yaxis.set_visible(False)

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
    else:
        # st()
        size = max(map.keys())
        x_min = 0
        x_max = (size[0]+1) * TILESIZE
        y_min = 0
        y_max = (size[1]+1) * TILESIZE
        #x_min, x_max, y_min, y_max = get_map_corners(map)
        ax.axis('equal')
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.xaxis.set_visible(False)
        ax.yaxis.set_visible(False)

        # fill in the road regions
        road_tiles = []
        width_tiles = np.arange(0,size[0]+1)*TILESIZE
        lanes_tiles = np.arange(0,size[1]+1)*TILESIZE

        for i in np.arange(0,size[0]+1):
            for k in np.arange(0,size[1]+1):
                if map[(i,k)] != '*':
                    tile = patches.Rectangle((width_tiles[k],lanes_tiles[i]),TILESIZE,TILESIZE,linewidth=1,facecolor='k', alpha=0.4)
                    road_tiles.append(tile)
        ax.add_collection(PatchCollection(road_tiles, match_original=True))

        # now add crosswalk on top
        crosswalk_tiles = []
        for item in CROSSWALK_LOCATIONS.keys():
            if item % 2 == 0:
                color = 'silver'
                alpha = 0.5
            else:
                color = 'k'
                alpha = 0.5
            width = CROSSWALK_LOCATIONS[item][1]*TILESIZE
            lanes = CROSSWALK_LOCATIONS[item][0]*TILESIZE
            tile = patches.Rectangle((width,lanes),TILESIZE,TILESIZE/2,linewidth=1,facecolor=color, alpha=alpha)
            crosswalk_tiles.append(tile)
        ax.add_collection(PatchCollection(crosswalk_tiles, match_original=True))

        plt.gca().invert_yaxis()

def draw_timestamp(t, merge = False):
    if merge:
        ax.text(0.5,0.7,t, transform=plt.gcf().transFigure,fontsize='large',
             bbox={"boxstyle" : "circle", "color":"white", "ec":"black"})
    else:
        ax.text(0.3,0.7,t, transform=plt.gcf().transFigure,fontsize='large',
             bbox={"boxstyle" : "circle", "color":"white", "ec":"black"})

def draw_pedestrian(ped_data):
    name, _, _, cwloc = ped_data
    x_tile = CROSSWALK_LOCATIONS[cwloc][1]
    y_tile = CROSSWALK_LOCATIONS[cwloc][0]
    x = (x_tile) * TILESIZE
    y = (y_tile) * TILESIZE - TILESIZE/2
    ped_fig = Image.open(ped_figure)
    ped_fig = ped_fig.rotate(180, expand=False)
    offset = 0.1
    ax.imshow(ped_fig, zorder=1, interpolation='bilinear', extent=[x+10, x+TILESIZE-10, y+2, y+TILESIZE-2])


def draw_car(car_data, merge = False):
    if merge:
        if car_data[0]=='system':
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
        ax.imshow(car_fig, zorder=1, interpolation='none', extent=[x+5, x+TILESIZE-5, y+5, y+TILESIZE-5])
    else:
        if car_data[0]=='ego':
            color = 'red'
        else:
            color = 'blue'
        # st()
        name, y_tile, x_tile, orientation = car_data
        theta_d = ORIENTATIONS[orientation]
        x = (x_tile) * TILESIZE
        y = (y_tile) * TILESIZE
        car_fig = Image.open(car_figs[color])
        car_fig = car_fig.rotate(theta_d, expand=False)
        offset = 0.1
        ax.imshow(car_fig, zorder=1, interpolation='bilinear', extent=[x+2, x+TILESIZE-2, y+2, y+TILESIZE-2])

def plot_sys_cars(agents):
    for i, agent in enumerate(agents):
        draw_car(agent)

def plot_tester_cars(agents):
    for i, agent in enumerate(agents):
        draw_car(agent)

def plot_peds(agents):
    for i,agent in enumerate(agents):
        draw_pedestrian(agent)

def draw_traffic_light(timestep):
    light = timestep % 15
    if light < 10:
        color = 'green'
    elif 10 <= light <= 12:
        color = 'yellow'
    else:
        color = 'red'
    theta_d = 180
    light_fig = Image.open(light_figs[color])
    light_fig = light_fig.rotate(theta_d, expand=False)
    offset = 0.1
    ax.imshow(light_fig, zorder=1, interpolation='bilinear', extent=[TILESIZE*5+17, TILESIZE*6-17, TILESIZE*2+5, TILESIZE*3-5])


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
        sys_agents = traces[t].ego
        tester_agents = traces[t].env
        ped_agents = traces[t].peds
        plot_sys_cars(sys_agents)
        plot_tester_cars(tester_agents)
        plot_peds(ped_agents)
        draw_timestamp(t)
        draw_traffic_light(t)
        plot_name = str(t).zfill(5)
        img_name = output_dir+'/plot_'+plot_name+'.png'
        fig.savefig(img_name)
    animate_images(output_dir)

def make_animation():
    output_dir = os.getcwd()+'/animations/gifs/'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    traces_file = os.getcwd()+'/intersection/saved_traces/sim_trace.p'
    traces_to_animation(traces_file, output_dir)

if __name__ == '__main__':
    make_animation()
