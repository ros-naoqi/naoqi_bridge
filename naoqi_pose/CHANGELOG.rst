^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_pose
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* bugfix: fixing python imports for nao_robot
* bugfix: python imports
* Contributors: Karsten Knese

0.5.0 (2014-11-06)
------------------
* moved pose manager into nao_robot
* transfer nao_robot
* 0.4.1
* update changelogs
* get the accent right in Séverin's name
* 0.4.0
* update changelogs
* 0.3.0
* update changelogs
* Fix xapparser __init__.py encoding, UTF used in header
* Moved the predefined pose support from pose_manager to nao_controller. Action name changed as well
* Support for predefined postures via actionlib
* "0.2.3"
* Changelogs
* {ahornung->ros-nao}
* {ahornung->ros-nao}
* "0.2.2"
* changelog
* Replace accented character in package.xml files, seems to cause
  problems with bloom
* "0.2.1"
* Changelogs
* "0.2.0"
* Adjust version number mismatch
* Adding (edited) catkin-generated changelogs
* Adding bugtracker and repo URLs to package manifests
* Fix xapparser.py encoding, UTF used in header:
  http://www.python.org/peps/pep-0263.html
* Small cleanup after many merges
* Merge pull request `#5 <https://github.com/ros-naoqi/nao_robot/issues/5>`_ from severin-lemaignan/minor-tweaks
  Minor tweaks in nao_pose
* [nao_pose] Added support for Choregraphe's XAP posture library
  This allows a user to create Nao posture from Choregraphe, export them to XAP files
  and load them in the pose library for later invokation with body_pose action.
* [nao_pose] Add a parser for Choregraphe XAP posture libraries
* [nao_pose] Remove useless build dependencies
* [nao_pose] Do not include nao_driver.launch in pose manager launch file
* [nao_pose] remove references to roslib
* Import pose_manager from nao_remote into nao_robot/nao_pose
* Contributors: Armin Hornung, Karsten Knese, Manos Tsardoulias, Miguel Sarabia, Séverin Lemaignan, Vincent Rabaud
