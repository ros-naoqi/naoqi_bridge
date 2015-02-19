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
# io_export_collision.py
# Authors: Mikael Arguedas [mikael.arguedas@gmail.com]

# This script import one by one each collada file in a folder,
# decimate the meshes and export them as stl files

from __future__ import print_function
import bpy
import os
import sys

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

mesh_dir = argv[0]
# Define decimation ratio
RATIO = float(argv[1])
# Define minimum number of vertices required for decimation
THRESHOLD = int(argv[2])
# Define output directory
output_dir = argv[3]
print("Exporting collision meshes to <%s>." % mesh_dir)

scene = bpy.context.scene
# Delete all existing objects in the current scene
for ob in scene.objects:
    ob.select = True

bpy.ops.object.delete()

file_list = sorted(os.listdir(mesh_dir))
for file in file_list:
    # Import the visual meshes one by one
    if file.endswith('.dae') == True:
        print(str(mesh_dir + '/' + file))
        bpy.ops.wm.collada_import(filepath= os.path.join(mesh_dir , file))
        # Decimate the meshes only if there are more than THRESHOLD meshes in it
        if(len(bpy.context.scene.objects[0].data.vertices) > THRESHOLD):
            bpy.ops.object.modifier_add(type='DECIMATE')
            mod = bpy.context.scene.objects.active.modifiers[0]
            mod.ratio = RATIO

            # Apply decimation
            bpy.ops.object.modifier_apply(apply_as='DATA')
        else:
            print (" does not have enough vertices for DECIMATION")
        # Export them
        bpy.ops.export_mesh.stl(filepath=os.path.join(output_dir,
                                                      file[0:file.find('.dae')]
                                                      + "_" + "{:.2f}".format(RATIO)+ ".stl"))
        # Delete them
        bpy.ops.object.delete()

bpy.ops.wm.quit_blender()
