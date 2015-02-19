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
# io_export_ogre.py
# Authors: Mikael Arguedas [mikael.arguedas@gmail.com]

# This script import one by one each collada file in a folder,
# Merge them when necessary and export them as .mesh files (OGRE)
# This is an Aldebaran specific scripts which won't work on blender files of other companies

from __future__ import print_function
import bpy
import os
import sys

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

mesh_dir = argv[0]
output_dir = argv[1]
print("Exporting ogre meshes to <%s>." % output_dir)

scene = bpy.context.scene

# Delete all existing objects in the blender file
for ob in scene.objects:
     ob.select = True
bpy.ops.object.delete()

# List the collada files present in the mesh_dir folder
file_list = sorted(os.listdir(mesh_dir))
for file in file_list:
    if file.endswith('.dae') == True :
        # Import the collada meshes one by one
        bpy.ops.wm.collada_import(filepath= os.path.join(mesh_dir , file))
        # Select all the objects and set the parent object as the active one
        for ob in scene.objects:
            if ob.type == 'MESH':
                ob.select = True
                if ob.name.find("_skip") ==-1:
                    bpy.context.scene.objects.active = ob
            else:
                ob.select = False
        # Merge the selected meshes
        bpy.ops.object.join()
        for ob in scene.objects:
            ob.name = file
        # Export the mesh as a .mesh file (OGRE)
        bpy.ops.ogre.export(filepath=os.path.join(output_dir, file) + ".scene",
        EX_COPY_SHADER_PROGRAMS=False,
        EX_MATERIALS = False,
        EX_SWAP_AXIS='xyz',
        EX_MESH=True,
        EX_MESH_OVERWRITE=True,
        EX_SCENE=False,
        EX_EXPORT_HIDDEN=False,
        EX_FORCE_CAMERA=False,
        EX_FORCE_LAMPS=False,
        EX_ARM_ANIM=False,
        EX_SHAPE_ANIM=False,
        EX_ARRAY=False,
        EX_optimiseAnimations=False)
        # Delete all the objects
        bpy.ops.object.select_all(action="SELECT")
        bpy.ops.object.delete()
        print ("\n\n removed all objects")
        for ob in scene.objects:
            print(ob.name)
        print ("\n\n")
        # Remove all the materials
        for material in bpy.data.materials:
            bpy.data.materials[material.name].user_clear()
            bpy.data.materials.remove(material)

bpy.ops.wm.quit_blender()


