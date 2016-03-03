#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from robot_class import *
from Tkinter import *

#This class handles resizing the GUI part of the canvas for demonstrative purposes
class ResizingCanvas(Canvas):
    def __init__(self,parent,**kwargs):
        Canvas.__init__(self,parent,**kwargs)
        self.bind("<Configure>", self.on_resize)
        self.height = self.winfo_reqheight()
        self.width = self.winfo_reqwidth()

    def on_resize(self,event):
        # determine the ratio of old width/height to new width/height
        wscale = float(event.width)/self.width
        hscale = float(event.height)/self.height
        self.width = event.width
        self.height = event.height
        # resize the canvas 
        self.config(width=self.width, height=self.height)
        # rescale all the objects tagged with the "all" tag
        self.scale("all",0,0,wscale,hscale)

#Method responsible for creating canvas
def createCanvas():
	global root
	myframe = Frame(root)
	myframe.pack(fill=BOTH, expand=YES)
	mycanvas = ResizingCanvas(myframe,width=103, height=103, bg="white", highlightthickness=0)
	mycanvas.pack(fill=BOTH, expand=YES)

# add some widgets to the canvas
	#vertical
	mycanvas.create_line(1, 1, 1, 101)
	mycanvas.create_line(101, 1, 101, 101)
	
	#horizontal
	mycanvas.create_line(1, 1, 101, 1)
	mycanvas.create_line(1, 101, 101, 101)
	

# tag all of the drawn widgets
	mycanvas.addtag_all("all")
	
#determining keyPressed and adjusting co-ordinates.
def keyPressed(c):

	if c == "A":
		rospy.loginfo("You pressed Up")
		goUp()	
	elif c == "B":
		rospy.loginfo("You pressed Down")
		goDown()

	elif c == "D":
		rospy.loginfo("You pressed Left")
		goLeft()

	elif c == "C":
		rospy.loginfo("You pressed Right")
		goRight()

	elif c == "1":
		rospy.loginfo("Exiting now.")
		rospy.shutdown()

	else:
		rospy.loginfo(c)

	
#set of functions responsible for changing coordinates and checking the walls.
def goUp():
	global current_y,max_y,current_x
	if (current_y + 1) == max_y:
		print "Can't go up"
		print "Current Coordinates: [%d , %d]"%(current_x,current_y)
	else:
		current_y = current_y + 1
		print "New Coordinates: [%d , %d]"%(current_x,current_y)

def goDown():
	global current_y,min_y,current_x
	if (current_y - 1) == min_y:
		print "Can't go Down"
		print "Current Coordinates: [%d , %d]"%(current_x,current_y)
	else:
		current_y = current_y - 1
		print "New Coordinates: [%d , %d]"%(current_x,current_y)

def goRight():
	global current_y,max_x,current_x
	if (current_x + 1) == max_x:
		print "Can't go Right"
		print "Current Coordinates: [%d , %d]"%(current_x,current_y)
	else:
		current_x = current_x + 1
		print "New Coordinates: [%d , %d]"%(current_x,current_y)

def goLeft():
	global current_y,min_x,current_x
	if (current_x - 1) == min_x:
		print "Can't go Left"
		print "Current Coordinates: [%d , %d]"%(current_x,current_y)
	else:
		current_x = current_x - 1
		print "New Coordinates: [%d , %d]"%(current_x,current_y)


def talker():
	createCanvas()
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(60) # 60hz
	while not rospy.is_shutdown():
	
	
		import termios, fcntl, sys, os
		fd = sys.stdin.fileno()

		oldterm = termios.tcgetattr(fd)
		newattr = termios.tcgetattr(fd)
		newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
		termios.tcsetattr(fd, termios.TCSANOW, newattr)

		oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
		fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

		try:
		    while 1:
			try:
				root.update()
				c = sys.stdin.read(1)
				hello_str = c
				
				keyPressed(c)
				
				pub.publish(hello_str)
				rate.sleep()
			except IOError: pass
		finally:
		    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
		    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

		print 'you entered', c

        

if __name__ == '__main__':
    try:	
	current_x = 1
	current_y = 1
	max_x = 100
	max_y = 100
	min_x = 0
	min_y = 0
	root = Tk()
	talker()
    except rospy.ROSInterruptException:
        pass







