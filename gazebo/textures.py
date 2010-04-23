#!/usr/bin/env python

import os

savedPath = os.getcwd()
if(not os.path.exists("Room.wld")):#we are running make texture from ../ directory
	os.chdir("gazebo")

f = open('./Media/materials/scripts/Gazebo.material', 'w')


dirList = os.listdir('./Media/materials/textures/')

for name in dirList:
	if(name[0] == '.'):
		continue
	f.write('\n')
	f.write('material Custom/'+name.split('.')[0]+'\n{\n')
	f.write('\ttechnique\n\t{\n')
	f.write('\t\tpass\n\t\t{\n')
	f.write('\t\t\ttexture_unit\n\t\t\t{\n')
	f.write('\t\t\t\ttexture '+name+'\n')
	f.write('\t\t\t}\n')
	f.write('\t\t}\n')
	f.write('\t}\n')
	f.write('}\n\n')

f.close()

os.chdir( savedPath )



