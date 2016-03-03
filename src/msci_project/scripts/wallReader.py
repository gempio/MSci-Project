#!/usr/bin/env python

file = open('/home/maciejm/catkin_ws/src/msci_project/map1','r')
i = 0
walls = []
for line in file:
	cords = line.rstrip('\n').split("-")
	cords1 = cords[0].split(",")
	x1 = cords1[0]
	y1 = cords1[1]
	cords2 = cords[1].split(",")
	x2 = cords2[0]
	y2 = cords2[1]
	walls.append([x1,y1,x2,y2])
	i = i + 1

print walls