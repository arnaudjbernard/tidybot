#!/usr/bin/python
# -*- coding: utf-8 -*-

# ECE 8853 - Autonomous Control of Robotic Systems
# Project - Tidyb
# Sethu Madhav Bhattiprolu && Arnaud BERNARD
# Spring 2010

# This file reads Room.wld and generates the Room.png

import Image, ImageDraw, os

def main():
	savedPath = os.getcwd()
	if(not os.path.exists("Room.wld")):#we are running make map from ../ directory
		os.chdir("map")
	RoomDef = open("Room.wld")
	size = (22, 22)
	obstacles = []
#	Beaconrange = 3
#	beacons = []
#	origin = (0,0)
	for Line in RoomDef.readlines() :
		Line = Line.strip()
		Tokens = Line.split(" ")
		Keyword = Tokens[0].strip()
		if Line == "" :
			continue
		elif Keyword == "MapSize" : # size of the image
			size = (int(Tokens[1].strip()), int(Tokens[2].strip()))
		elif Keyword == "Obstacle" : # Obstacle (x, y, width, height)
			obstacles.append((	1 + int(Tokens[1].strip()), \
								1 + int(Tokens[2].strip()), \
								int(Tokens[1].strip()) + int(Tokens[3].strip()), \
								int(Tokens[2].strip()) + int(Tokens[4].strip())))
#		elif Keyword == "Beacon" : # Obstacle (x, y, width, height)
#			beacons.append(( int(Tokens[1].strip()), int(Tokens[2].strip())))
#		elif Keyword == "Origin" : # size of the image
#			origin = (float(Tokens[1].strip()), float(Tokens[2].strip()))
#		elif Keyword == "Beaconrange" : # beacons range
#			Beaconrange = int(Tokens[1].strip())
	print "Obstacles : " + str(obstacles)
	print "Map size : " + str(size)
#	print "Robot starts at : " + str(origin)
	
	im = Image.new('L', size, 0)
	draw = ImageDraw.Draw(im)
	draw.rectangle((1, 1, size[0]-2, size[1]-2), fill = 255);
	for obstacle in obstacles:
		draw.rectangle(obstacle, fill=0)
	Path = "../gazebo/Media/materials/textures/Room.png"
#	im = im.transpose(Image.FLIP_TOP_BOTTOM)
	im = im.rotate(90)
	im.save(Path, 'PNG')
	print "Image \"" + Path + "\" has been successfully created."
	
	
#	fi = open("beacons.txt", "w")
#	fi2 = open("beacons.inc", "w")
#	for beacon in beacons:
#		draw.point(beacon, fill=222)
#		fi.write(str(beacon[0])+"\n"+str(beacon[1])+"\n")
#		fi2.write('zone(name "beacon"size [1.414 1.414 0.00]pose ['+str(beacon[0])+' '+str(beacon[1])+' 0 0])')
#	fi2.close()
#	fi.close()
#	fi = open("origin.txt", "w")
#	fi.write(str(origin[0])+"\n"+str(origin[1])+"\n")
#	fi.write(str(size[0])+"\n"+str(size[1])+"\n")
#	fi.write(str(Beaconrange)+"\n")
#	fi.close()
#	fi = open("origin.inc", "w")
#	fi.write('  pose [' + str(origin[0] + 0.5) + ' ' + str(origin[1] + 0.5) + ' 0 0]\n')
#	fi.close()
#	fi = open("size.inc", "w")
#	fi.write('  size [' + str(size[0]) + ' ' + str(size[1]-1.0) + ' 1.00]\n')
#	fi.write('  pose [' + str((size[0])/2.0) + ' ' + str((size[1]-1.0)/2) + ' 0 0]\n')
#	fi.close()
	os.chdir( savedPath )

if __name__ == "__main__":
    main() 
