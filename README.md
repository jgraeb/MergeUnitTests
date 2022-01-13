# Compositional Scenario Testing
## Authors and Contributors
Josefine B. Graebener, Apurva Badithela <br />

## Description
Framework to merge unit tests for autonomous vehicles using a receding horizon winning set filter synthesized from the merged test specification and Monte Carlo Tree Search.

This repository reproduces the results in the paper submitted to NFM 2022
This branch contains code for reproducing the results in "Towards Better Test Coverage: Merging Unit Tests for Autonomous Systems" by Josefine B. Graebener, Apurva Badithela, and Richard M. Murray submitted to NFM 2022.

## Requirements
Python 3.x<br />
Packages: see **requirements.txt** and TuLiP <br />

## Instructions
1. Install the required packages by running 'pip install -r requirements.txt' <br />
2. Install TuLiP from https://github.com/tulip-control/tulip-control
3. Run sim_merge.py for a demonstration of the lane change example <br />
4. Run sim_intersection.py for a demonstration of the unprotected left turn at intersection example
5. To produce an animation of the result run animate.py after step 3. or 4.
6. For the runtime evaluations run runtime_tests.py with the desired flag for MCTS or winning set synthesis. Detailed instructions are at the top of the file.

## Examples
### Highway Merge
![](animations/merge_track10.gif)
### Unprotected Left Turn at Intersection
![](animations/intersection_animation.gif)
