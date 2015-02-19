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
# normalize_meshes.py
# Authors: Mikael Arguedas [mikael.arguedas@gmail.com]

# Normalize all the matrix element in the collada files in a given folder

import argparse
from xml.dom.minidom import parse, parseString
import os

parser = argparse.ArgumentParser(usage='convert offset of all dae files in a directory')
parser.add_argument('-i','--input', default=None, help='inputDirectory')
parser.add_argument('-o','--output', default=None, help='outputDirectory')
parser.add_argument('-s','--scale', default=1, help='scale factor for the meshes')

args = parser.parse_args()
# Check if the input directory is valid
directory = args.input
output = args.output

# Open every collada file
file_list = sorted(os.listdir(directory))
for file in file_list:
    if file.endswith('.dae'):
        dom = parse(directory + '/' + file)
        # Look for matrix element in the collada files and replace them by a scales Identity transformation matrix
        for node in dom.getElementsByTagName('matrix'):
            if node.nodeName == 'matrix':
                node.firstChild.nodeValue = args.scale + " 0 0 0 0 " + args.scale + " 0 0 0 0 " + args.scale + " 0 0 0 0 1"
        # Put relative path for textures (based on aldebaran ros packages architecture)
        for node in dom.getElementsByTagName('init_from'):
            if node.firstChild.nodeValue.startswith('/'):
                node.firstChild.nodeValue= '../../texture/' +str(node.firstChild.nodeValue[node.firstChild.nodeValue.rfind('/')+1:])
        print 'processing ' + file
        f = open(os.path.join(directory, file),'w+')
        # Write the modified xml file
        f.write(dom.toxml())
        f.close()

