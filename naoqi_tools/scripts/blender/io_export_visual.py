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
# io_export_visual.py
# Authors: Mikael Arguedas [mikael.arguedas@gmail.com]
#
# This script parses a blender file from Aldebaran robotics, export all the materials of the scene,
# It also export all the meshes to COLLADA files (.dae)

from __future__ import print_function
import bpy
import os
import sys

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

# Initialize the blacklist:
# If you don't want to export some meshes, either add their name to this list
# of suffix them with a keyword and add the keyword to this list
blacklist = ["skip"]

# Get the folder where the meshes will be saved
mesh_dir = argv[0]
name = argv[1]

# Keep a copy of user selection
bpy.ops.object.select_by_type(type="MESH")
sel_obs = bpy.context.selected_objects[:]

# Export all the materials from the scene
bpy.ops.ogre.export(filepath=os.path.join(mesh_dir, name + '.scene'),
EX_COPY_SHADER_PROGRAMS=False,
EX_MATERIALS = True,
EX_SEP_MATS=False,
EX_SWAP_AXIS='xyz',
EX_MESH_OVERWRITE=False,
EX_MESH=False,
EX_SELONLY=False,
EX_SCENE=False,
EX_EXPORT_HIDDEN=False,
EX_FORCE_CAMERA=False,
EX_FORCE_LAMPS=False,
EX_ARM_ANIM=False,
EX_SHAPE_ANIM=False,
EX_ARRAY=False,
EX_optimiseAnimations=False)

# Browse the object list and export all the meshes as COLLADA files(.dae)
for ob in bpy.data.objects:

    skip_mesh = False
    for keyword in blacklist:
        if ob.name.find(keyword) != -1:
            skip_mesh = True
    # Skip non-mesh objects or all children meshes we don't want to export
    if ob.type != 'MESH' or skip_mesh == True:
        continue

    # Clear selection
    bpy.ops.object.select_all(action="DESELECT")

    # Select single object
    ob.hide = False
    ob.select = True


    # Export single object to COLLADA file (.dae)
    bpy.context.scene.collada_export(filepath=os.path.join(mesh_dir, ob.name + ".dae"), selected = True, include_uv_textures=True, include_children=True)


# Restore user selection
bpy.ops.object.select_all(action="DESELECT")
for ob in sel_obs:
    ob.select = True
bpy.context.scene.objects.active = ob

print("%s meshes exported." % len(sel_obs))

bpy.ops.wm.quit_blender()
