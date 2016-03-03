#!/usr/bin/env python
#This class holds a constructor for the robots so that they can be initialized as much as possible.
class Robot:
	def __init__(self,cur_x,cur_y,energy):
		self.cur_x = cur_x
		self.cur_y = cur_y
		self.energy = energy
