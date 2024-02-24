# MAPF with Precedence Constraints

This repo contains the implementations of CBS-PC forked from HanZhang39/MAPF-PC. Please refer to their paper for more details about the problem formulation and algorithm details.
In this forked repo, I have updated the algorithm to meet my need for my Integrated Task and Motion Planner.
1. The first goal can be same as start position. I have used it so that, I can force some robots to stay in their position if they are not assigned any task.
2. Added custom deadlines for each goals.
3. Removed pbs and greedy task planner. 
4. Added runMaStar to test MA* algorithm
5. Added nholman json to read problem statement directly from json file.
6.  

To begin with, you can compile the project using the following commands

``` shell
cmake .
make
```
, and Cmake will generate three executables in the `bin` folder.
They are:

1. `cbs` runs the CBS-PC algorithm.
3. `runMaStar`  executes MA* algorithm

For each executables, you can type `--help` to see the expected input arguments it. 
You can find example in `run_exp.sh`.
The `input_sample` folder contains sample input files.
