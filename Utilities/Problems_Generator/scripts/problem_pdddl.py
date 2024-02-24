import os
import sys
from random import randint
import networkx as nx
import random

def generate_pddl(pddl_file, map, robots, tasks):
	pddl_file.write("(define (problem warehouse-01)\n")
	pddl_file.write("\t(:domain warehouse)\n")
	pddl_file.write("\t\t(:objects\n\t\t\t")
	for i in range(len(robots)):
		pddl_file.write("r"+str(i)+" ")
	pddl_file.write("- robot\n\t\t\t")
	for i in range(len(map)):
		for j in range(len(map[i])):
			pddl_file.write("l"+str(i).zfill(2)+str(j).zfill(2)+" ")
	pddl_file.write("- location\n\t\t\t")
	for i in range(len(tasks)):
		pddl_file.write("t"+str(i)+" ")
	pddl_file.write("- task\n\t\t\t\n\t\t)\n")
	pddl_file.write("\t\t(:init\n")
	for i in range(len(map)):
		for j in range(len(map[i])):
			if i > 0:
				pddl_file.write("\t\t\t(adjecent l"+str(i).zfill(2)+str(j).zfill(2)+" "+"l"+str(i-1).zfill(2)+str(j).zfill(2)+")\n")
			if j > 0:
				pddl_file.write("\t\t\t(adjecent l"+str(i).zfill(2)+str(j).zfill(2)+" "+"l"+str(i).zfill(2)+str(j-1).zfill(2)+")\n")
			if i < len(map)-1:
				pddl_file.write("\t\t\t(adjecent l"+str(i).zfill(2)+str(j).zfill(2)+" "+"l"+str(i+1).zfill(2)+str(j).zfill(2)+")\n")
			if j < len(map)-1:
				pddl_file.write("\t\t\t(adjecent l"+str(i).zfill(2)+str(j).zfill(2)+" "+"l"+str(i).zfill(2)+str(j+1).zfill(2)+")\n")

	for i in range(len(map)):
		for j in range(len(map[i])):
			if map[i][j] == 1:
				pddl_file.write("\t\t\t(blocked l"+str(i).zfill(2)+str(j).zfill(2)+")\n")

	for i in range(len(robots)):
		pddl_file.write("\t\t\t(robot-at r"+str(i)+" l"+str(robots[i][0]).zfill(2)+str(robots[i][1]).zfill(2)+")\n")
		pddl_file.write("\t\t\t(blocked l"+str(robots[i][0]).zfill(2)+str(robots[i][1]).zfill(2)+")\n")
		
	for i in range(len(tasks)):
		pddl_file.write("\t\t\t(task-at t"+str(i)+" l"+str(tasks[i][0][0]).zfill(2)+str(tasks[i][0][1]).zfill(2)+")\n")
		pddl_file.write("\t\t\t(blocked l"+str(tasks[i][0][0]).zfill(2)+str(tasks[i][0][1]).zfill(2)+")\n")
		pddl_file.write("\t\t\t(is-task-goal-location t"+str(i)+" l"+str(tasks[i][1][0]).zfill(2)+str(tasks[i][1][1]).zfill(2)+")\n")

	pddl_file.write("\t\t\n\t\t(:goal\n\t\t(and\n")
	for i in range(len(tasks)):
		pddl_file.write("\t\t\t(task-at t"+str(i)+" l"+str(tasks[i][1][0]).zfill(2)+str(tasks[i][1][1]).zfill(2)+")\n")
	pddl_file.write("\t\t)\n\t\t)\n)")

def read_pddl(problem_file, no_robots, no_goals):
	mapsize = int(problem_file.readline())
	map = [[0 for i in range(mapsize)] for j in range(mapsize)]
	robots = []
	tasks = []
	for i in range(mapsize):
		line = problem_file.readline()
		jj = 0
		for j in range(1, 3*mapsize, 3):
			if int(line[j]) == 1:
				map[i][jj] = 1
			jj+=1
	'''for i in map:
		print i'''
	problem_file.readline()
	problem_file.readline()
	for i in range(no_robots):
		x = int(problem_file.readline())
		y = int(problem_file.readline())
		problem_file.readline()
		robots.append([x, y])

	problem_file.readline()
	problem_file.readline()
	for i in range(no_goals):
		x1 = int(problem_file.readline())
		y1 = int(problem_file.readline())
		problem_file.readline()
		x2 = int(problem_file.readline())
		y2 = int(problem_file.readline())
		problem_file.readline()
		tasks.append([[x1, y1], [x2, y2]])

	#print robots
	#print tasks
	return map, robots, tasks

def main():

	for rob_no in [3]:
		for goals_no in [5]:
			mapsizes =  [20, 30, 40, 50, 60, 70, 80, 90, 100]
			for mapsize in mapsizes:
				for i in range(20):
					print(str(rob_no)+":"+str(goals_no)+":"+str(mapsize)+":"+str(i))
					directory = "problems/"+str(mapsize)+"/"+str(rob_no)+"/"+str(goals_no)
					pddl_directory = "pddl_problems/"+str(mapsize)+"/"+str(rob_no)+"/"+str(goals_no)+"/p"+str(i).zfill(2)
					if not os.path.exists(pddl_directory):
						os.makedirs(pddl_directory)
					problem_file = open(directory+"/problem_"+str(i).zfill(2), "r")
					pddl_file = open(pddl_directory+"/problem.pddl","w+")
					map, robots, tasks = read_pddl(problem_file, rob_no, goals_no)
					generate_pddl(pddl_file, map, robots, tasks)

if __name__ == '__main__':
	main()