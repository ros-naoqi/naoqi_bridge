^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package naoqi_pose
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.5 (2016-09-19)
------------------

0.5.4 (2016-05-20)
------------------

0.5.3 (2015-08-26)
------------------
* add services for life
* Contributors: Karsten Knese

0.5.2 (2015-08-11)
------------------

0.5.1 (2015-07-31)
------------------
* generate changelog
* generate changelog
* rename naoqi_msgs to naoqi_bridge_msgs
* Contributors: Karsten Knese

* generate changelog
* rename naoqi_msgs to naoqi_bridge_msgs
* Contributors: Karsten Knese

* rename naoqi_msgs to naoqi_bridge_msgs
* Contributors: Karsten Knese

0.5.0 (2015-07-30)
------------------
* delete legacy msg package
* Contributors: Karsten Knese

0.4.8 (2015-06-25)
------------------
* fix package name/version for nao_pose -> naoqi_pose
* properly install Python scripts
  This fixes `#19 <https://github.com/ros-naoqi/naoqi_bridge/issues/19>`_
* add wakeup and rest services
* remove trailing spaces
* bugfix: python imports
* moved pose manager into nao_robot
* transfer nao_robot
* get the accent right in Séverin's name
* Fix xapparser __init__.py encoding, UTF used in header
* Moved the predefined pose support from pose_manager to nao_controller. Action name changed as well
* Support for predefined postures via actionlib
* {ahornung->ros-nao}
* Replace accented character in package.xml files, seems to cause
  problems with bloom
* Adjust version number mismatch
* Adding bugtracker and repo URLs to package manifests
* Fix xapparser.py encoding, UTF used in header:
  http://www.python.org/peps/pep-0263.html
* Small cleanup after many merges
* [nao_pose] Added support for Choregraphe's XAP posture library
  This allows a user to create Nao posture from Choregraphe, export them to XAP files
  and load them in the pose library for later invokation with body_pose action.
* [nao_pose] Add a parser for Choregraphe XAP posture libraries
* [nao_pose] Remove useless build dependencies
* [nao_pose] Do not include nao_driver.launch in pose manager launch file
* [nao_pose] remove references to roslib
* Import pose_manager from nao_remote into nao_robot/nao_pose
* Contributors: Armin Hornung, Karsten Knese, Kei Okada, Manos Tsardoulias, Miguel Sarabia, Mikael ARGUEDAS, Séverin Lemaignan, Vincent Rabaud
