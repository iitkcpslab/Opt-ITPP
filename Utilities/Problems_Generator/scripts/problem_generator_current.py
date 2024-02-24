import os
import sys
import json
from random import randint
import networkx as nx
import random
from random import randrange
from random import shuffle


# Load a predefined map.
def load_world(location, ipos_count = 0):
    with open(location, 'r') as fp:
        try:
            data = json.load(fp)
        except Exception as e:
            raise Exception('Incorrect map size given')
    world_descriptor =  data['world_descriptor']
    bases = list(map(tuple, data['bases']))
    ipos = list(map(tuple, data['intermediate_positions']))
    
    if ipos_count > 0:
        available_locations = []
        for i in range(len(world_descriptor)):
            for j in range(len(world_descriptor[0])):
                if world_descriptor[i][j] == 0 and (i, j) not in bases and (i, j) not in ipos:
                    available_locations.append( tuple([i, j]) )
               
        shuffle(available_locations)
        for i in range( max(0, ipos_count - len(ipos) ) ):
            ipos.append( available_locations[i] )
    return world_descriptor, bases, ipos[:ipos_count]

# Create a new map config
def generate_world(size, seed=0, obstacle=0, ipos_count=0):
    world_descriptor = [[0 for i in range(size[1])] for j in range(size[0])]
    random.seed(seed)
    bases = [(0, size[1]//2), (size[0]-1, size[1]//2), (size[0]//2, 0), (size[0]//2, size[1]-1), ]
    checkset = []
    for i in range(size[0]):
        for j in range(size[1]):
            if random.uniform(0, 1) < obstacle and (i, j) not in bases:
                world_descriptor[i][j] = 1
            elif (i, j) not in bases:
                checkset.append( (i, j) )
    random.shuffle(checkset)
    ipos = []
    for i in range(ipos_count):
        index = randint(0, len(checkset) - 1)
        ipos.append(checkset[index])
        checkset.pop(index)

    return world_descriptor, bases, ipos

# Generate a problem given a world config
def generate_itmp_problem(world, bases, ipos, rob_no, tasks_no, seed, task_weight=5):
    random.seed(seed)
    rob_capacity = task_weight * randint(2, 4)
    
    # Create a checkset to get available Locations
    checkset = []
    for i in range(len(world)):
        for j in range(len(world[0])):
            if world[i][j] == 0 and (i, j) not in bases and (i, j) not in ipos:
                checkset.append( (i, j) )
    
    # Generate robots and their capacities from checkset
    robots = []
    capacities = []
    for i in range(rob_no):
        index = randint(0, len(bases) - 1)
        robots.append( bases[index] )
        bases.pop(index)
        capacities.append( task_weight * randint(2, max(3, tasks_no)) )
    
    # Create tasks, their weight and deadlines.
    tasks = []
    for i in range(tasks_no):
        task = []
        index = randint(0, len(checkset) - 1)
        task.append( checkset[index] )
        checkset.pop(index)
        
        index = randint(0, len(checkset) - 1)
        task.append( checkset[index] )
        checkset.pop(index)
        
        minRand = max(5, tasks_no-2)
        task.append( (len(world) + len(world[0])) * randint(minRand - 1, max(minRand, tasks_no)) )
        task.append( task_weight )
        tasks.append(task)    
            
    return {
        'world_descriptor' : world,
        'robots' : robots,
        'tasks' : tasks,
        'capacities' : capacities,
        'intermediate_positions' : ipos,
    }

def generate_path_planning_problem(world, num_tasks, tasks_no, seed):
    random.seed(seed)
    
    location, map_loc, rev_map_loc = [], {}, {}
    counter = 1
    checkset = []
    for i in range(len(world)):
        for j in range(len(world[0])):
            if world[i][j] == 0:
                checkset.append( (i, j) )
    
    for i in range(tasks_no * num_tasks):
        index = randint(0, len(checkset) - 1)
        loc =  checkset[index]
        checkset.pop(index)
        
        map_loc[counter] = loc
        rev_map_loc[loc] = counter
        location.append(loc)
        counter += 1
    
    objectives = []
    goals = []
    actions = []
    tasks_numbers = []
    locs = []
    for i in range(num_tasks):
        objective, goal, action = [], [], []
        objective = location[ i*tasks_no: (i+1)*tasks_no ]
        obj = []
        for val in objective:
            goal.append( rev_map_loc[val] )
             
        if tasks_no % 2 == 1:
            for j in range(tasks_no):
                if j == 0:
                    action.append(-1)
                else:
                    if j % 2:
                        action.append( [rev_map_loc[objective[j]], 0] )
                    else:
                        action.append( [rev_map_loc[objective[j-1]], 1] )
                        
            for j in range(tasks_no):
                if action[j] == -1:
                    obj.append( (objective[j], 'SF') )
                else:
                    if j % 2:
                        obj.append( (objective[j], 'D' + str(action[j]) ) )
                    else:
                        obj.append( (objective[j], 'P' + str(action[j]) ) )
        else:
            for j in range(tasks_no):
                if j == 0:
                    action.append(-1)
                elif j == tasks_no - 1:
                    action.append([rev_map_loc[objective[j]], 0])
                else:
                    if j % 2:
                        action.append( [rev_map_loc[objective[j]], 0] )
                    else:
                        action.append( [rev_map_loc[objective[j-1]], 1] )
            
            for j in range(tasks_no):
                if j == 0:
                    obj.append( (objective[j], 'SF') )
                elif j == tasks_no - 1:
                    obj.append( (objective[j], 'FI') )
                else:
                    if j % 2:
                        obj.append( (objective[j], 'P' + str(action[j]) ) )
                    else:
                        obj.append( (objective[j], 'D' + str(action[j]) ) )
                        
            
        objectives.append(obj)
        goals.append(goal)
        actions.append(action)
        
    # return objectives, goals, actions, map_loc, rev_map_loc
    return {
        'world_descriptor' : world,
        'objectives' : objectives,
        'goals' : goals,
        'actions' : actions,
        'all_pos' : map_loc
    }