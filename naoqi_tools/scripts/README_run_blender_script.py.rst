run_ogre_blender_script
=======================

Description
--------------
This script is a launcher for the python scripts located in the blender folder.
These scripts are now Aldebaran conventions agnostic.
Blender doesn't support argparse library, this launcher allow to pass arguments
in a standard way and it takes care of converting them and launch the blender
scripts accordingly.

Dependencies
-------------
 * Blender2.71 or higher
 * Blender add_ons: "io_export_selected.py" and "io_export_ogreDotScene.py" (available in the blender folder of
   this package)

The export of the meshes to OGRE format (.mesh) requires OgreXMLConverter program to be installed

.. code-block:: bash

    sudo apt-get install ogre-1.8-tools


How to use it
--------------
There is a different cet of parameters for each script. Only 2 of them are used
by every script:
* --scriptfile or -s : string : name of the blender script you want to call (in
  this case io_export_visual.py)
* --outputdir or -o : string : absolutee path of the directory to export the
  meshes and material
  If it doesn't exists it will be created
  If not provided, the files will be exported in the directory provided as input

io_export_visual.py
______________________
Loads a blender file and export all the materials to a single .material file
Export one by one all the meshes of the scene except those containing a
keyword listed in the blacklist (io_export_visual.py line34)


Parameters:
* --blenderfile or -f : string : absolute path of the .blend file you want to
  process
* --blenderdir or -b : string : absolute path of your blender installation directory
  : default : /usr/bin/
* --outputfilename or -n : string : name of the exported .material file
  If not provided the material file will have the same name as the blender file
  provided in --blenderfile or -f
* --outputdir or -o : string : absolutee path of the directory to export the
  meshes and material
  If it doesn't exists it will be created
  If not provided, the files will be exported in the directory of the .blend file
* --scriptfile or -s : string : name of the blender script you want to call (in
  this case io_export_visual.py)

io_export_ogre.py
______________________
Load one by one every collada (.dae) file in the folder and export them to OGRE
format (.mesh)


Parameters:
* --blenderdir or -b : string : absolute path of your blender installation directory
  : default : /usr/bin/
* --inputmeshdir or -i : string : absolute path of the folder containing the
   COLLADA (.dae) files to convert
* --outputdir or -o : string : absolutee path of the directory to export the
  meshes and material
  If it doesn't exists it will be created
  If not provided, the files will be exported in the directory provided as input (-i or --inputmeshdir)
* --scriptfile or -s : string : name of the blender script you want to call (in
  this case io_export_ogre.py)

NOTE: ogre exporter plugin exports object as MESHNAME.ogre. So the name of the
resulting file may be different of the name of the blender object exported.

normalise_meshes.py
______________________
Open one by one every collada(.dae) file in a folder, set the translation from
origin to 0 and apply scaling


Parameters:
* --scale : float : scale to apply to the transformation of all the meshes in
  the collada files
* --inputmeshdir or -i : string : absolute path of the folder containing the
   COLLADA (.dae) files to convert
* --outputdir or -o : string : absolute path of the directory to export the
  normalized collada meshes
  If it doesn't exists it will be created
  If not provided, the files will be exported in the directory provided as input (-i or --inputmeshdir)
* --scriptfile or -s : string : name of the blender script you want to call (in
  this case normalise_meshes.py)


io_export_collision.py
______________________
Import one by one every collada(.dae) file in a folder, decimate them and export
tham as <FILENAME>_<DECIMATION_RATIO>.stl
Parameters:


* --ratio or -r: float : ratio of the decimation to apply before generating
  collision files
  : default : 0.1 (keep only 10% of the vertices)
* --threshold or -t: int : minimum number of vertices to apply decimation (it
  doesn't make sens to decimate very small meshes)
  : default : 50
* --blenderdir or -b : string : absolute path of your blender installation directory
  : default : /usr/bin/
* --inputmeshdir or -i : string : absolute path of the folder containing the
   COLLADA (.dae) files to decimate
* --outputdir or -o : string : absolute path of the directory to export the
  STL collision meshes
  If it doesn't exists it will be created
  If not provided, the files will be exported in the directory provided as input (-i or --inputmeshdir)
* --scriptfile or -s : string : name of the blender script you want to call (in
  this case io_export_collision.py)



TODO
-------------
Create script to convert any type of file to any other type supported by blender (not only COLLADA and OGRE)
