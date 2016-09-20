^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pepper_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2015-03-31)
------------------
* fix install rules
* call catkin macro in the right order
* use the proper name for naoqi_sensors
* Contributors: Vincent Rabaud

0.0.1 (2015-03-31)
------------------
* POD: Use PoseWithConfidenceStamped messages.
* RVIZ: Add naoqi_rviz plugins and pod display plugin.
* BLINDZONES: Fix multiple arrays issues.
* NAVIGATION: Add Tracker node.
* NAVIGATION: Add naoqi prefix to node names.
* NAVIGATETO: Transform points in /base_footprint frame.
* NAVIGATION: Send navigateTo on /initialpose messages.
* remove duplicate moveto node
* NAVIGATION: Add move to listener to send moveTo via rviz.
* POD: Publish pod confidence only only if position is published.
* NAVIGATION: Fix setup.py.
* NAVIGATION: Fix occupancy maps and and grid cells.
* NAVIGATION: Fix safe map display.
* NAVIGATION: Add secure map display.
* NAVIGATION: Fix navigation_sensors to display colored point cloud.
* NAVIGATION: Set robot data in base_footprint frame.
* Add navigation sensors displaying blind zones data.
* NAVIGATION: Fix blind zones display.
* NAVIGATION: Stop nodes when they are not subscribed.
* NAVIGATION: Add navigation package.
* Contributors: Karsten Knese, lsouchet
