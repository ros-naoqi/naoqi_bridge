^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naoqi_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
