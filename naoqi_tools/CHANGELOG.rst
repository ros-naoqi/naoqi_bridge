^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naoqi_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2015-08-26)
------------------
* add add_dummy_collision function for gazebo simulation
* Contributors: Mikael Arguedas

0.5.2 (2015-08-11)
------------------

0.5.1 (2015-07-31)
------------------
* generate changelog
* generate changelog
* Contributors: Karsten Knese

* generate changelog
* Contributors: Karsten Knese

0.5.0 (2015-07-30)
------------------
* make sure we add the cap for Romeo
* fix bad macro names
* Contributors: Vincent Rabaud

0.4.8 (2015-06-25)
------------------
* also fix the right laser frames
* add an executable to compare xacro files
* add xmldiff to compare XML files
* add specific laser frames for Pepper and misc bugfixes
* use the conventional functions for mesh processing
* Contributors: Vincent Rabaud

0.4.7 (2015-03-30)
------------------

0.4.6 (2015-02-27)
------------------
* Update README_run_blender_script.py.rst
* fix for issue 31: to be tested
* update wiki ros links
* update repo links in package.xml
* upload blender scripts
* add optical frames for each camera
* fix typo for inversion of sonars
* Use catkin_install_python macro
* Move generate_urdf.py system bin dir to package bin dir
* Contributors: Arguedas Mikael, Mikael Arguedas, Takashi Ogura, Vincent Rabaud

0.4.5 (2015-02-11)
------------------

0.4.4 (2015-01-16)
------------------

0.4.3 (2014-12-14)
------------------
* naoqui_tools not depending on robot descriptions
  I was gonna test Nao in ROS and, hopefully, in Gazebo. I didn't find any updated guide on that so I'm just debugging as I work.
  I did a:
  rosdep install --from-paths src --ignore-src --rosdistro hydro -y
  On a workspace with only ```nao_robot``` and ```naoqi_bridge``` and I got:
  ERROR: the following packages/stacks could not have their rosdep keys resolved to system dependencies:
  naoqi_tools: Cannot locate rosdep definition for [romeo_description]
  As it's named "tools" I think that maybe it shouldn't depend on the robots descriptions (or you'll need all of them in the workspace!).
* remove _actuator suffix to Hand joints
* Contributors: Mikael ARGUEDAS, Sammy Pfeiffer

0.4.2 (2014-11-26)
------------------
* remove useless dependency
* Remove suffix _joint for all joints
* get package version to agree
* add changelogs
* fix module dependency to comply new git organization
* update file according to comments of vrabaud
* changed actuator frames naming convention
* Update README.rst
* add urdf parsing libs and generation script
* Contributors: Arguedas Mikael, Mikael ARGUEDAS, Vincent Rabaud, margueda

* fix module dependency to comply new git organization
* update file according to comments of vrabaud
* changed actuator frames naming convention
* Update README.rst
* add urdf parsing libs and generation script
* Contributors: Arguedas Mikael, Mikael ARGUEDAS, margueda

0.4.1 (2014-11-13)
------------------

0.4.0 (2014-11-06)
------------------
