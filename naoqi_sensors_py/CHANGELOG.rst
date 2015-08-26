^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naoqi_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2015-08-26)
------------------
* remove useless dependencies
* Contributors: Vincent Rabaud

0.5.2 (2015-08-11)
------------------
* naoqi_sensors_py no longer contains nodelt plugins
* OCTOMAP: Use binaryMsg for compliance with other octree types.
* SONAR: Fix Sonar FOV.
* Contributors: Kei Okada, lsouchet

0.5.1 (2015-07-31)
------------------
* generate changelog
* generate changelog
* Contributors: Karsten Knese

* generate changelog
* Contributors: Karsten Knese

0.5.0 (2015-07-30)
------------------
* renaming packag to _py
* change package dependencies to *_py
* delete legacy msg package
* rename packages
* remove cpp version
* remove camera nodelet
* rename packages to *_py
* Contributors: Karsten Knese

0.4.8 (2015-06-25)
------------------
* Missing dependency of libpython
* hotfix: complete cast of alimage into ros::time
* Use different ros time constructor
* Fix time conversion in cpp camera driver
  The timestamp of the image and diagnostic messages from the cpp driver of the
  camera contains "random" data.
  The Aldebaran doc [1,2] states that element 4 contains seconds and element 5
  contains microseconds. In the code these are swapped. Also see the python
  node where the conversion is done correctly.
  [1] http://doc.aldebaran.com/2-1/naoqi/vision/alvideodevice-api.html#image
  [2] http://doc.aldebaran.com/1-14/naoqi/vision/alvideodevice-api.html#image
* remove Groovy compatibility
* Contributors: Jon Dybeck, Karsten Knese, Kenta Yonekura, Vincent Rabaud

0.4.7 (2015-03-30)
------------------
* install octomap_python
* Contributors: Vincent Rabaud

0.4.6 (2015-02-27)
------------------
* update repo links in package.xml
* use proper optical frames for the robot
* Contributors: Mikael Arguedas, Vincent Rabaud

0.4.5 (2015-02-11)
------------------
* Update nodelet names in log messages
* Update names in nodelet launchfiles
* Update camera nodelet names in comments
* Update the xml for the camera nodelet
  During the restructuring the nodelet was renamed, but the metadata in the
  xml file was not updated. This breaks loading of the nodelet. Here the
  metadata is updated. Packages that use the nodelet will need to update
  their launchfiles.
* SENSORS: Publish octomap and sonars only when subscribed.
  Signed-off-by: lsouchet <lsouchet@aldebaran-robotics.com>
* remove debug line
* add raw depth support
* Contributors: Jon Dybeck, Vincent Rabaud, lsouchet

0.4.4 (2015-01-16)
------------------
* cleanup some installs
* change default 3d frame
* configure sonars via external params
  launch multiple sonars via diff namespaces
  see launching cameras for this
* Contributors: Karsten Knese, Vincent Rabaud

0.4.3 (2014-12-14)
------------------
* Merge pull request `#22 <https://github.com/ros-naoqi/naoqi_bridge/issues/22>`_ from Karsten1987/hotfix_camera_info
  bugfix: correctly load camera calibration acc. to cam source
* use 16UC1 as kinect do
* delete print, introduce parantheses if statement
* bugfix: correctly load camera calibration acc. to cam source
* Fix camera node for NAO version 3.2 with Naoqi 1.14. Changed the camProxy.subscribe() which was deprecated to the subscribeCamera, and as the call was the same for both versions removed the if for the version. Also changed the auto_Exposure_algo parameter to 0 as the parameters 2 and 3 were not valid for this NAO model (it gave an error), so parameter 0 and 1 should work for every NAO.
* Added missing rule for NaoqiCameraConfig
  Because it said the file naoqi_sensors/NaoqiCameraConfig.h was missing
* get the packages to actually conflict with the old versions (nao*)
  The replace tag does not provide a way to uninstall the packages.
  Its use case is different.
* Contributors: Gerard Canal, Karsten Knese, Kei Okada, Vincent Rabaud

0.4.2 (2014-11-26)
------------------
* Install launchfiles even if naoqi not found
* update changelogs
* Merge pull request `#5 <https://github.com/ros-naoqi/naoqi_bridge/issues/5>`_ from jondy276/add-support-for-local-broker
  Added support for connecting to local broker
* Try remote naoqi before local naoqi.
* Added support for connecting to local broker
  By doing this the nodelet can connect to a local broker allowing
  direct function calls and memory access.
  Note that the nodelet will prefer the local broker over one
  specified on the command line or using ros parameters.
* Fix namespace problem in naoqi_camera
  The new method getNaoqiParams did not take into account the namespace
  change done using repo restructuring. Here the correct namespace is
  used for the method.
* Contributors: Arguedas Mikael, Jon Dybeck, Vincent Rabaud

* Merge pull request `#5 <https://github.com/ros-naoqi/naoqi_bridge/issues/5>`_ from jondy276/add-support-for-local-broker
  Added support for connecting to local broker
* Try remote naoqi before local naoqi.
* Added support for connecting to local broker
  By doing this the nodelet can connect to a local broker allowing
  direct function calls and memory access.
  Note that the nodelet will prefer the local broker over one
  specified on the command line or using ros parameters.
* Fix namespace problem in naoqi_camera
  The new method getNaoqiParams did not take into account the namespace
  change done using repo restructuring. Here the correct namespace is
  used for the method.
* Contributors: Jon Dybeck, Vincent Rabaud

0.4.1 (2014-11-13)
------------------
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
