import os
import sys
from random import randint
import networkx as nx
import random
import json

nx.exception.NetworkXNoPath


def genMap(mapsize):
    if (mapsize // 10) % 2 == 0:
        x =  ([ (1, 1) ] +  [ (i, i+1) for i in range(4, mapsize, 4)] + [ (mapsize, mapsize)] )
        y = []
        for i in range(0, mapsize, 10):
            y.extend( [ (i+2, i+9)] * len(x) ) 
        x = x * (mapsize//10)

    else:
        
        x =  [ (i, i+1) for i in range(3, mapsize, 4)]
        y = []
        for i in range(0, mapsize, 10):
            y.extend( [ (i+2, i+9)] * len(x) ) 
        x = x * (mapsize//10)

    tmap = [ [0] * mapsize for i in range(mapsize)]
    for xr, yr in zip(y, x):
        for i in range(xr[0]-1, xr[1]):
            for j in range(yr[0]-1, yr[1]):
                tmap[i][j] = 1
    
    return tmap


def test(mapsize, index, rob_no, tasks_no, intermediate_pos_no, seed):
	random.seed(seed)
	directory = "problems/"+str(mapsize)+"/"+str(rob_no)+"/"+str(tasks_no)
	problem_file = open(directory+"/problem_"+str(index).zfill(2), "w")
	#seed1 = randint(0, sys.maxsize)
	#seed2 = randint(0, sys.maxsize)
	#seed3 = randint(0, sys.maxsize)
	#random.seed(seed1)
	world_descriptor = genMap(mapsize)
	
	# offset = 4
	# for i in range(0,19):
	# 	for j in range(3,25):
	# 			world_descriptor[offset+i*5][j] = 1
	# 			world_descriptor[offset+i*5+1][j] = 1
	# 	for j in range(74,96):
	# 			world_descriptor[offset+i*5][j] = 1
	# 			world_descriptor[offset+i*5+1][j] = 1

	# offset = 31
	# for i in range(0,6):
	# 	for j in range(4,35):
	# 			world_descriptor[j][offset+i*3+1] = 1
	# 			world_descriptor[j][offset+i*3] = 1
	# 	for j in range(65,96):
	# 			world_descriptor[j][offset+i*3+1] = 1
	# 			world_descriptor[j][offset+i*3] = 1

	# offset = 52
	# for i in range(0,6):
	# 	for j in range(4,35):
	# 			world_descriptor[j][offset+i*3+1] = 1
	# 			world_descriptor[j][offset+i*3] = 1
	# 	for j in range(65,96):
	# 			world_descriptor[j][offset+i*3+1] = 1
	# 			world_descriptor[j][offset+i*3] = 1

	allocated = []
	robots = []
	for i in range(rob_no):
		x = randint(0, mapsize-1)
		y = randint(0, mapsize-1)
		while [x,y] in allocated or world_descriptor[x][y] == 1:
			x = randint(0, mapsize-1)
			y = randint(0, mapsize-1)
		robots.append([x, y])
		allocated.append([x,y])
		world_descriptor[x][y] = 0

	tasks = []
	for j in range(tasks_no):
		task = []
		x = randint(0, mapsize-1)
		y = randint(0, mapsize-1)

		while [x,y] in allocated or world_descriptor[x][y] == 0:
			x = randint(0, mapsize-1)
			y = randint(0, mapsize-1)

		task.append([x, y])
		world_descriptor[x][y] = 0
		allocated.append([x,y])
		world_descriptor[x][y] = 0

		x = randint(0, mapsize-1)
		y = randint(0, mapsize-1)
		while [x,y] in allocated or world_descriptor[x][y] == 0:
			x = randint(0, mapsize-1)
			y = randint(0, mapsize-1)

		task.append([x, y])
		world_descriptor[x][y] = 0
		allocated.append([x,y])
		world_descriptor[x][y] = 0

		tasks.append(task)


	problem_file.write(str(mapsize)+"\n")
	for i in world_descriptor:
		problem_file.write(str(i)+"\n")
	problem_file.write("\nRobots: \n")
	for i in robots:
		problem_file.write(str(i[0])+"\n"+str(i[1])+"\n"+"\n")
	problem_file.write("\nTasks: \n")
	for i in tasks:
		for j in i:
			problem_file.write(str(j[0])+"\n"+str(j[1])+"\n"+"\n")

	dstore = {
		'world_descriptor' : world_descriptor,
		'tasks' : tasks,
		'robots' : robots
	}
	with open(directory+"/problem_"+str(index).zfill(2)+'.json', 'w+') as fp:
		json.dump(dstore, fp)
		

def main():
	seeds = []
	seedfile = open("seeds.seed","r")
	for mapsize in [20, 30, 40, 50, 60, 70, 80, 90, 100]:
		for rob_no in [3]:
			for tasks_no in [5]:
				for line in seedfile:
					seeds.append(int(line))
				for i in range(20):
					print(str(rob_no)+":"+str(tasks_no)+":"+str(mapsize)+":"+str(i))
					directory = "problems/"+str(mapsize)+"/"+str(rob_no)+"/"+str(tasks_no)
					if not os.path.exists(directory):
						os.makedirs(directory)
					test(mapsize, i, rob_no, tasks_no, 0, seeds[i])
	'''for z in Z:
		for line in seedfile:
			seeds.append(int(line))
		for i in range(20):
			print(i)
			
			test(i,1,1,seeds[i],z)'''

if __name__ == '__main__':
	main()