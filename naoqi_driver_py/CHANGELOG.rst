^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naoqi_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.5 (2016-09-19)
------------------
* add doc to args
* disable odom with param and get arg from launch file
* set Surya as the maintainer
* cmd_vel and move_base_simple/goal topic names renamed without '/' at first
* parametrize topic names
* Contributors: Igor Rodriguez, Kei Okada, Naoki-Kameyama, Vincent Rabaud

0.5.4 (2016-05-20)
------------------
* Merge branch 'add_fake_effort' of https://github.com/k-okada/naoqi_bridge into k-okada-add_fake_effort
* Contributors: Vincent Rabaud

0.5.3 (2015-08-26)
------------------
* remove useless dependencies
* Contributors: Vincent Rabaud

0.5.2 (2015-08-11)
------------------
* really remove the .xml for diagnostics
* Contributors: Vincent Rabaud

0.5.1 (2015-07-31)
------------------
* generate changelog
* generate changelog
* Contributors: Karsten Knese

* generate changelog
* Contributors: Karsten Knese

0.5.0 (2015-07-30)
------------------
* update package.xml description
* remove wrongly set conflict tag
* Merge branch 'naoqi_py'
* delete legacy msg package
* rename packages
* remove cpp library builds
* remove cpp code
* rename packages to <*>_py
* Contributors: Karsten Knese

0.4.8 (2015-06-25)
------------------
* remove Groovy compatibility
* Contributors: Vincent Rabaud

0.4.7 (2015-03-30)
------------------
* MOVETO: transform moveTo commands in /base_footprint frame.
* Contributors: lsouchet

0.4.6 (2015-02-27)
------------------
* update repo links in package.xml
* fix the naoqi_logger name
* fix bad nao_logger package
* readd logger
* Contributors: Mikael Arguedas, Vincent Rabaud

0.4.5 (2015-02-11)
------------------
* DRIVER: Add node to send motion moveTo via rviz.
* propper install of the nodes
* Contributors: Vincent Rabaud, lsouchet

0.4.4 (2015-01-16)
------------------

0.4.3 (2014-12-14)
------------------
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
