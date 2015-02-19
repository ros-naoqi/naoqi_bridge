generate_urdf.py
================

Description
-----------
This script allows to convert an official Aldebaran URDF file into a REP120 compliant set of xacro files.
Official Aldebaran urdf files should be included in Aldebaran documentation very soon.
It is an Aldebaran specific script which will certainly not work on other robots. I handles every aldebaran robot : hence <ROBOT> can be any of the following : romeo, nao and pepper

Prerequisites
-------------
This script assumes that you have the <ROBOT>_description package installed, it will export all the files in <ROBOT>_description/urdf/<ROBOT><VERSION>_generated_urdf/

URDF parsing
------------
This code doesn't use urdfdom because it was lacking xacro, comments and gazebo tags support. Some bugs and features about handling materials were also problematic. The impossibility to create xml custom nodes by hand was a blocking issue so we decided to use a minidom base urdf parsing library.


Run the script
---------------
.. code-block:: bash

	rosrun naoqi_tools generate_urdf.py -i <PATH_TO_URDF_FILE> [-x urdf]

If you want to build a single URDF file with the complete model, you can either add the parameter -x urdf or build an urdf file frome the xacro files using:

.. code-block:: bash

	roscd <ROBOT>_description/urdf/<ROBOT><VERSION>_generated_urdf/
    	rosrun xacro xacro.py <ROBOT>_robot.xacro > <ROBOT>.urdf

You can display the model in RVIZ using:

.. code-block:: bash

   	roslaunch urdf_tutorial display.launch gui:=true model:=$(rospack find <ROBOT>_description)/urdf/<ROBOT><VERSION>_generated_urdf/<ROBOT>.urdf

TODO
----
Parsing lib:
 * Factorize sensor classes in gazebo parsing library
 * Find a generic way to export nodes as string (remove huge __str__ functions)
 * Complete/Update gazebo library: handle latest sdf conventions and all sensor supported by gazebo

Script:
 * Add Gazebo and Transmission automatic exports (once Aldebaran libs allow it)
 * Add r/l_toe frames to romeo model (not supported by NAOqi models yet)
 * Export gaze in a generic way (remove offsets from dictionaries)
 * Add xacro property for using the cap on romeo or not


