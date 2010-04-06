#!/usr/bin/env python

import os

f = open('./Media/materials/scripts/Gazebo.material', 'w')


dirList = os.listdir('./Media/materials/textures/')

for name in dirList:
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





