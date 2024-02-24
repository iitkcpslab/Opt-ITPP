import matplotlib.pyplot as plt
import numpy as np
import random


def find_direction(x, y, map):
	if map[x-1][y] == 0:
		return '<'
	if map[x+1][y] == 0:
		return '>'
	if map[x][y-1] == 0:
		return 'v'
	if map[x][y+1] == 0:
		return '^'


def draw(map, robots, tasks, output):
	blocked_x = []
	blocked_y = []
	for i in xrange(len(map)):
		for j in xrange(len(map[i])):
			if(map[i][j] == 1):
				blocked_x.append(i);
				blocked_y.append(j);

	robots_x = []
	robots_y = []
	for i in xrange(len(robots)):
		robots_x.append(robots[i][0])
		robots_y.append(robots[i][1])

	pickup_x = []
	pickup_y = []
	drop_x = []
	drop_y = []
	for i in xrange(len(tasks)):
		pickup_x.append([tasks[i][0][0]])
		pickup_y.append([tasks[i][0][1]])
		drop_x.append([tasks[i][1][0]])
		drop_y.append([tasks[i][1][1]])


	fig = plt.figure()
	ax = fig.add_subplot(111)
	
	ax.plot(blocked_x, blocked_y, 'ks', markersize = 1)
	ax.plot(robots_x, robots_y, 'ro', markersize=2)
	random.randint(1,2)
	rgb = random.randint(100000, 999999)
	for i in xrange(len(pickup_x)):
		shape = find_direction(pickup_x[i][0], pickup_y[i][0], map)
		ax.scatter(pickup_x[i][0], pickup_y[i][0], s=4, c='#'+str(rgb), marker = shape)
		shape = find_direction(drop_x[i][0], drop_y[i][0], map)
		ax.scatter(drop_x[i][0], drop_y[i][0], s=4, c='#'+str(rgb), marker = shape)
		rgb = random.randint(100000, 999999)
	
	ax.axis("off")
	ax.set_aspect('equal', 'box')
	plt.xticks([])
	plt.yticks([])
	plt.savefig(output+'.png', dpi=1000)


def read_pddl(problem_file, no_robots, no_goals):
	mapsize = int(problem_file.readline())
	map = [[0 for i in xrange(mapsize)] for j in xrange(mapsize)]
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
	filename = "problem"
	problem_file = open(filename)
	map, robots, tasks = read_pddl(problem_file, 3, 5)
	draw(map, robots, tasks, "graph1")





if __name__ == '__main__':
	main()
