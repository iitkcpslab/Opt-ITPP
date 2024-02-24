import numpy as np
from random import randint

def generate(filename, rob_no, tasks_no):
	mapsize = 100
		
	problem_file = open(filename, "w+")
	world_descriptor = [[0 for i in xrange(mapsize)] for j in xrange(mapsize)]
	
	offset = 4
	for i in xrange(0,19):
		for j in xrange(3,25):
				world_descriptor[offset+i*5][j] = 1
				world_descriptor[offset+i*5+1][j] = 1
		for j in xrange(74,96):
				world_descriptor[offset+i*5][j] = 1
				world_descriptor[offset+i*5+1][j] = 1

	offset = 31
	for i in xrange(0,6):
		for j in xrange(4,35):
				world_descriptor[j][offset+i*3+1] = 1
				world_descriptor[j][offset+i*3] = 1
		for j in xrange(65,96):
				world_descriptor[j][offset+i*3+1] = 1
				world_descriptor[j][offset+i*3] = 1

	offset = 52
	for i in xrange(0,6):
		for j in xrange(4,35):
				world_descriptor[j][offset+i*3+1] = 1
				world_descriptor[j][offset+i*3] = 1
		for j in xrange(65,96):
				world_descriptor[j][offset+i*3+1] = 1
				world_descriptor[j][offset+i*3] = 1

	

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

def main():
	generate("problem", 3, 5)


if __name__ == '__main__':
	main()



		