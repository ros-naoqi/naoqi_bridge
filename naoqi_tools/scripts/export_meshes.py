#! /usr/bin/env python

# Copyright (C) 2014 Aldebaran
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# export_meshes.py
# Authors: Mikael Arguedas [mikael.arguedas@gmail.com]

# This script opens a blender file, exports all meshes to collada, merges every mesh group and exports them as .mesh file (OGRE), removes the offset from the colladas, decimates them and exports collision meshes in STL format. Finally cleans your output directory of every .xml files
# This is an Aldebaran specific scripts. It certainly won't work on blender files of robots from other companies

from __future__ import print_function
import os
import argparse
import subprocess

parser = argparse.ArgumentParser(usage='Export meshes and convert them')
parser.add_argument('-b','--blenderdir', default='/usr/bin', help='location of your blender directory')
parser.add_argument('-f','--file', default='nao-v4.blend',help='full path of the blender file to process')
parser.add_argument('-o','--outputmeshdir',default=None, help='directory to export the meshes to')

args = parser.parse_args()

if os.path.basename(args.file).lower().startswith('nao'):
    robot='nao'
    version='V40'
    suffix = '_meshes'
    scale = 0.01
elif os.path.basename(args.file).lower().startswith('juliette') or os.path.basename(args.file).lower().startswith('pepper') :
    robot='pepper'
    version = ''
    suffix = '_description'
    scale = 0.01
elif os.path.basename(args.file).lower().startswith('romeo'):
    robot='romeo'
    version = ""
    suffix = '_description'
    scale = 1
else:
    print("robot name unknown")
    exit(1)
package = robot + suffix

if args.outputmeshdir == None :
    print("\nno valid output directory, looking for " + package + " ROS package\n")
    cmd= 'rospack find ' + package
    path_meshes = subprocess.check_output(cmd, stderr=subprocess.STDOUT, shell=True)[:-1]
    if not path_meshes:
      print('package "' + path_meshes + '" not found')
else:
    if not os.path.isdir(args.outputmeshdir):
      print('creating the output folder because it does not exist')
      os.makedirs(args.outputmeshdir)
    path_meshes = args.outputmeshdir

extractor_path = subprocess.check_output('rospack find naoqi_tools', stderr=subprocess.STDOUT, shell=True)[:-1]
script_path = os.path.join(extractor_path , 'scripts', 'blender')

if version:
    path_meshes = os.path.join(path_meshes, version)

print("extractor path :" + extractor_path)

# Export meshes as collada files and all the materials of the scene
os.system('./run_blender_script.py -s io_export_visual.py -o ' + path_meshes + ' -f ' + args.file)

# Import collada files one by one and export each of them as a single .mesh(OGRE) file
os.system('./run_blender_script.py -s io_export_ogre.py -i ' + path_meshes)

# Normalize exported collada meshes to give them the right scale and orientation
os.system('./run_blender_script.py -s normalize_meshes.py -i ' + path_meshes + ' --scale ' + str(scale))

# Import all collada meshes, decimate them and export them as stl files
os.system('./run_blender_script.py -s io_export_collision.py -i ' + path_meshes)

# Remove files left by the OGRE exporter
file_list = sorted(os.listdir(path_meshes))
for file in file_list:
    if file.endswith('.mesh.xml') or file.endswith('.mesh') or file.endswith('.material'):
        print('removing ' + file)
        os.remove(os.path.join(path_meshes , file))
