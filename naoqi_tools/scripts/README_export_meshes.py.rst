export_meshes.py
================
This is an Aldebaran specific scripts. It certainly won't work on blender files of robots from other companies
Please check README_run_blender_script.py.rst for more general purposes use.

Description
-------------

This script proceed as follows:
* opens a blender file
* exports all meshes to collada
* merges every mesh group into a single mesh and exports them as .mesh file (OGRE),
* removes the offset from the colladas,
* decimates them and exports collision meshes in STL format.
* Finally cleans your output directory of every .xml files

How it works
-------------
This script calls sequentially all the scripts in th blender folder

Dependencies
-------------
 * Blender2.71 or higher
 * Blender add_on: io_export_selected.py (available in the blender folder of
   this package)

The export of the meshes to OGRE format (.mesh) requires OgreXMLConverter program to be installed

.. code-block:: bash

    sudo apt-get install ogre-1.8-tools

And the blender add_on io_export_ogreDotScene.py

