#! /usr/bin/env python

# Copyright (C) 2014 Mikael Arguedas
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
# run_blender_script.py
# Authors: Mikael Arguedas [mikael.arguedas@gmail.com]

# This script is a launcher for every blender script available in naoqi_tools
# It defines a set of parameter and format them to sned the right bash commands
# to Launch blender with the chosen script and the right arguments

from __future__ import print_function
import os
import argparse
import subprocess

parser = argparse.ArgumentParser(usage='Export meshes and convert them')
parser.add_argument('-s','--scriptfile', default='io_export_collision.py',
                    help='name of the blender script to Launch')
parser.add_argument('-b','--blenderdir', default='/usr/bin',
                    help='location of your blender directory')
parser.add_argument('-i','--inputmeshdir',default=None,
                    help='directory where collada meshes are located')
parser.add_argument('-o','--outputdir',default=None,
                    help='directory to export the meshes to')
parser.add_argument('-r','--ratio',default=0.1,
                    help='float value, ratio used to decimate meshes',
                    type=float)
parser.add_argument('-t','--threshold', default=50,
                    help='integer: minimum number of vertices for decimation',
                    type=int)
parser.add_argument('--scale', default = 1,
                    help='scale to resize the collada meshes to',type=float)
parser.add_argument('-n','--outputfilename',default=None,
                    help='name of the output file (for material file)')
parser.add_argument('-f','--blenderfile',default=None,
                    help='path of the blender file to process')

args = parser.parse_args()

clean = False

def check_output_dir(out):
    if not os.path.isdir(out):
        print('creating the output folder because it does not exist')
        os.makedirs(out)
    return out

# Create path to script
script_path = subprocess.check_output('rospack find naoqi_tools',
                                      stderr=subprocess.STDOUT, shell=True)[:-1]
script_path = os.path.join(script_path, 'scripts','blender',args.scriptfile)
# Check script existence
if(not os.path.isfile(script_path)):
    print("script doesn't exist\nExiting now")
    exit(1)

# Check if args.outputdir is valid
if args.scriptfile != 'io_export_visual.py':
    if args.outputdir == None :
        print("\nno valid output directory: using " + str(args.inputmeshdir) +
              " as destination folder")
        output_dir = args.inputmeshdir
    else:
      output_dir = check_output_dir(args.outputdir)

    # Check existence of the input directory
    if args.inputmeshdir == None or not os.path.isdir(args.inputmeshdir):
        print("Invalid mesh folder provided\nExiting now")
        exit(1)
else:
    if args.outputdir == None :
        output_dir = os.path.dirname(args.blenderfile)
        print("\nno valid output directory: using " +
              output_dir + " as destination folder")
    else:
        output_dir = check_output_dir(args.outputdir)

# Set the parameters and bash command for each script
if(args.scriptfile == 'normalize_meshes.py'):
    cmd = (script_path + ' -i ' + args.inputmeshdir + ' -s ' +
           str(args.scale) + ' -o ' + output_dir)

elif(args.scriptfile == 'io_export_collision.py'):
    cmd = (args.blenderdir + '/blender --background -P '+ script_path +
           ' -- ' + args.inputmeshdir + ' ' + str(args.ratio) + ' '
           + str(args.threshold) + ' ' + output_dir)

elif(args.scriptfile == 'io_export_visual.py'):
    if(args.blenderfile == None or not os.path.isfile(args.blenderfile)):
        print("invalid blender file provided\nExiting now")
        exit(1)
    if(args.outputfilename == None):
        basename = os.path.basename(args.blenderfile)
        print("no name specified for output material file. unsing: " +
              basename)
    else:
        basename = os.path.basename(args.outputfilename)
    if(basename.rfind('.')!= -1):
        basename = basename[0:basename.rfind('.')]
    print("exporting material to " + basename + ".material")
    cmd = (os.path.join(args.blenderdir,"blender") + ' ' + args.blenderfile +
           ' -P ' + script_path + ' -- ' + output_dir
           + ' ' + basename)

elif(args.scriptfile == 'io_export_ogre.py'):
    cmd = ((os.path.join(args.blenderdir,"blender")) + " -P " + script_path
           + " -- " + args.inputmeshdir + ' ' + output_dir)
    clean = True

os.system(cmd)

# If ogre exporter called
# Remove files left behind by the OGRE exporter
if clean == True:
    file_list = sorted(os.listdir(output_dir))
    for file in file_list:
        if file.endswith('.mesh.xml'):
            print('removing ' + file)
            os.remove(os.path.join(output_dir , file))
