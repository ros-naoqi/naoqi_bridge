#!/usr/bin/env python

# Copyright (C) 2014 Aldebaran Robotics
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

# This file takes two folders and compares the .xacro files in there
# it orders the XML to have proper XML comparisons (it deals with elements
# out of order basically)

from __future__ import print_function
import argparse
import os
from xmldiff.xmldiff import compareFiles

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compares all the .xacro in two folders.')
    parser.add_argument('folder1')
    parser.add_argument('folder2')
    args = parser.parse_args()
    files1 = set([ i for i in os.listdir(args.folder1) if i.endswith('.xacro')])
    files2 = set([ i for i in os.listdir(args.folder2) if i.endswith('.xacro')])
    for i in files1.intersection(files2):
        diff = compareFiles(os.path.join(args.folder1, i), os.path.join(args.folder2, i))
        if diff:
            print('---------------------------------------------------------')
            print('File %s in common' % i)
            print('---------------------------------------------------------')
            print(diff)
