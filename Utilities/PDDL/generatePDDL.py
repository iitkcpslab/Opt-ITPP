import json
import argparse

def generate_pddl(writeLocation, readLocation, offsetZ):

    pddl_file = open(writeLocation, 'w+')
    with open(readLocation, 'r') as fp:
        data = json.load(fp)
    world_descriptor = data['world_descriptor']
    tasks = data['tasks']
    robots = data['robots']
    capacities = data['capacities']
    intermediate_positions = data['intermediate_positions']


    pddl_file.write("(define (problem warehouse-01)\n")
    pddl_file.write("\t(:domain warehouse)\n")
    
    pddl_file.write("\t\t(:objects\n")
    pddl_file.write("\t\t\t")
    
    for i in range(len(world_descriptor)):           # Write the map in objects
        for j in range(len(world_descriptor[i])):
            pddl_file.write("l"+str(i).zfill(2)+str(j).zfill(2)+" ")
        pddl_file.write("- location\n")
        pddl_file.write("\t\t\t")
    for i in range(len(robots)):                     # Write the robots in objects
        pddl_file.write("r"+str(i)+" ")
    pddl_file.write("- robot\n")
    pddl_file.write("\t\t\t")
    for i in range(len(tasks)):                      # Write the ids of tasks present
        pddl_file.write("t"+str(i)+" ")
    pddl_file.write("- task\n")
    pddl_file.write("\t\t)\n")
    
    
    pddl_file.write("\t\t(:init\n")                  # Write the initial configuration
    
    for i in range(len(world_descriptor)):           # Write the adjacency predicates
        for j in range(len(world_descriptor[i])):
            if i > 0:
                pddl_file.write("\t\t\t(adjecent l"+str(i).zfill(2)+str(j).zfill(2)+" "+"l"+str(i-1).zfill(2)+str(j).zfill(2)+")\n")
            if j > 0:
                pddl_file.write("\t\t\t(adjecent l"+str(i).zfill(2)+str(j).zfill(2)+" "+"l"+str(i).zfill(2)+str(j-1).zfill(2)+")\n")
            if i < len(world_descriptor)-1:
                pddl_file.write("\t\t\t(adjecent l"+str(i).zfill(2)+str(j).zfill(2)+" "+"l"+str(i+1).zfill(2)+str(j).zfill(2)+")\n")
            if j < len(world_descriptor)-1:
                pddl_file.write("\t\t\t(adjecent l"+str(i).zfill(2)+str(j).zfill(2)+" "+"l"+str(i).zfill(2)+str(j+1).zfill(2)+")\n")

    for i in range(len(world_descriptor)):
        for j in range(len(world_descriptor[i])):
            if world_descriptor[i][j] == 1:
                pddl_file.write("\t\t\t(blocked l"+str(i).zfill(2)+str(j).zfill(2)+")\n")
   
    for i in range(len(robots)):
        pddl_file.write("\t\t\t(robot-at r"+str(i)+" l"+str(robots[i][0]).zfill(2)+str(robots[i][1]).zfill(2)+")\n")
        pddl_file.write("\t\t\t(blocked l"+str(robots[i][0]).zfill(2)+str(robots[i][1]).zfill(2)+")\n")
        pddl_file.write("\t\t\t(= (robot-capacity r"+str(i)+") "+str(capacities[i])+")\n")
        pddl_file.write("\t\t\t(= (robot-time r"+str(i)+") 0)\n")
        pddl_file.write("\t\t\t(= (robot-action r"+str(i)+") 0)\n")

    for i in range(len(tasks)):
        pddl_file.write("\t\t\t(task-at t"+str(i)+" l"+str(tasks[i][0][0]).zfill(2)+str(tasks[i][0][1]).zfill(2)+")\n")
        pddl_file.write("\t\t\t(is-task-goal-location t"+str(i)+" l"+str(tasks[i][1][0]).zfill(2)+str(tasks[i][1][1]).zfill(2)+")\n")
        pddl_file.write("\t\t\t(= (task-deadlines t"+str(i)+") "+str(tasks[i][2])+")\n")
        pddl_file.write("\t\t\t(= (task-weight t"+str(i)+") "+str(tasks[i][3])+")\n")


    pddl_file.write('\t\t\t(= (total-time) 0)\n')
    if len(tasks) % len(robots):
        minZ  = max(1, 2 + 2 * (len(tasks)//len(robots)) ) 
    else:
        minZ  = max(1, 2 * (len(tasks)//len(robots)) )
    minZ += offsetZ
    pddl_file.write('\t\t\t(= (maxstep) ' + str(minZ) +')\n')
    


    for pos in intermediate_positions:
        pddl_file.write("\t\t\t(is-intermediate-drop-location l"+str(pos[0]).zfill(2)+str(pos[1]).zfill(2)+")\n")

    pddl_file.write('\t\t)\n')
 
    
    pddl_file.write("\t\t(:goal\n\t\t\t(and\n")
    for i in range(len(tasks)):
        pddl_file.write("\t\t\t\t(task-at t"+str(i)+" l"+str(tasks[i][1][0]).zfill(2)+str(tasks[i][1][1]).zfill(2)+")\n")
    for i in range(len(robots)):
        pddl_file.write("\t\t\t\t(robot-at r"+str(i)+" l"+str(robots[i][0]).zfill(2)+str(robots[i][1]).zfill(2)+")\n")

    pddl_file.write("\t\t\t)\n")
    pddl_file.write("\t\t)\n")
    
    pddl_file.write('\t\t(:metric\n')
    pddl_file.write('\t\t\tminimize (total-time)\n')
    pddl_file.write('\t\t)\n')
    pddl_file.write("\n)")

    pddl_file.close()

    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("config_file")
    parser.add_argument("output_file")
    parser.add_argument("offsetZ")
    args = parser.parse_args()

    generate_pddl(args.output_file, args.config_file, args.offsetZ)