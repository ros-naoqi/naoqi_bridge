^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naoqi_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* put data into effort
* publish joint_stiffness
* also install the include folder
* install the FindNAOqi.cmake and use extra file in all cases
* get the packages to actually conflict with the old versions (nao*)
  The replace tag does not provide a way to uninstall the packages.
  Its use case is different.
* Contributors: Kei Okada, Vincent Rabaud

0.4.2 (2014-11-26)
------------------
* update changelogs
* Fix SDK dependencies (did not compile in C++ with every SDK versions)
* Added support for connecting to local broker
  By doing this the nodelet can connect to a local broker allowing
  direct function calls and memory access.
  Note that the nodelet will prefer the local broker over one
  specified on the command line or using ros parameters.
* Contributors: Jon Dybeck, Vincent Rabaud, sambrose

* Fix SDK dependencies (did not compile in C++ with every SDK versions)
* Added support for connecting to local broker
  By doing this the nodelet can connect to a local broker allowing
  direct function calls and memory access.
  Note that the nodelet will prefer the local broker over one
  specified on the command line or using ros parameters.
* Contributors: Jon Dybeck, sambrose

0.4.1 (2014-11-13)
------------------
* added speech dynamic reconfigure
* bugfix: naoqi migration
* Init ros node before getting parameters
  Fix a problem where the pip and pport parameters are always set to their
  default values when using the parameter server and not command line arguments.
* Search for free port for broker
  Setting the local port to zero causes the broker to search for a free port.
  In nao_robot/nao_apps/nao_tactile.py this technique is used to make the broker
  search for a free port. However this does not appear to be a documented
  by aldebaran.
* Update for naoqi version < 2.0
* Contributors: Edgar Riba, Jon Dybeck, Karsten Knese

0.4.0 (2014-11-06)
------------------
* fix install file for cmake hook
* removed wrong install routine
* introduce replace tag in package.xml
* cleanup
* moved to pose_controller.py in nao_robot
* quickfix for renaming
* renamed naoqi_sensors
* cmake extras hooks
* api change
* remove and renaming
* renamed subfolders for naoqi_*
* Contributors: Karsten Knese
