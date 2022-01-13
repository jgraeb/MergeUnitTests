# Compositional Scenario Testing
## Authors and Contributors
Apurva Badithela, Josefine B. Graebener <br />

## Description
Framework to merge unit tests for autonomous vehicles using a winning set filter synthesized from the combined test specification and Monte Carlo Tree Search.

## Requirements
Python 3.x<br />
Packages: see **requirements.txt** and TuLiP <br />

## Instructions
1. Install the required packages by running 'pip install -r requirements.txt' <br />
2. Install TuLiP from https://github.com/tulip-control/tulip-control
3. Run sim_merge.py for a demonstration of the highway merge <br />
4. Run sim_intersection.py for a demonstration of the highway merge
5. To produce a video of the result run animate.py after step 3. or 4.

## Example
### Highway Merge
![](animations/merge_track10.gif)
### Unprotected Left Turn
![](animations/intersection_animation.gif)
