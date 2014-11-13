^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naoqi_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#6 <https://github.com/ros-naoqi/naoqi_bridge/issues/6>`_ from jondy276/implement-critical-section-in-nodelet
  Implement critical section in nodelet
* OCTOMAP: Fix publisher for empty octree.
* bugfix: naoqi migration
* Fix sonar node
* add args for camera parameters
* expose configurable node_name in constructor
* excluded most of the sensors as a python module
  python files inside the node folder just contain a main function launching the module
  Conflicts:
  naoqi_sensors/nodes/microphone.py
* exclude microphone
* fixing imports on microphone sensor
* fixing imports on microphone sensor
* excluded sonar module
* excluded camera as a python module
* Added support for configuring ip and port of parent broker
  This commit adds support to the nao camera nodelet to get the
  ip and port settings from ros parameters. That way the nodelet
  can be configured using launchfiles or using command line parameters via ros.
* Remove old member and add comment
* Replace volatile bool with boost::mutex
  Volatile bool is not sufficient to prevent poll() and reconfigure() from
  executing concurrently. This causes the nodelet to crash when poll()
  releases the image on a closed camera.
  Here a mutex is introduced to guard the relevant critical sections.
* Contributors: Jon Dybeck, Karsten Knese, Vincent Rabaud, lsouchet, nao, sambrose

0.4.0 (2014-11-06)
------------------
* fix the version in order to bump everything
* Add 2 methods for extract and set camera parameters
* introduce replace tag in package.xml
* resolved imports
* renamed naoqi_sensors
* naoqi_sensors transfer
* removed or renamed wrongly placed files in naoqi_sensors
* renamed subfolders for naoqi_*
* Contributors: Karsten Knese, Vincent Rabaud, mchamoux
