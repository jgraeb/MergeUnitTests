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
