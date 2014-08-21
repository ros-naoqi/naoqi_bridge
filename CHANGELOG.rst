^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nao_sensors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* install the sensor scripts in the local bin
* Contributors: Vincent Rabaud

0.1.2 (2014-08-21)
------------------
* fix a compile bug on Groovy
  absolute path is not specified in the Findoctomap.cmake
* Contributors: Vincent Rabaud

0.1.0 (2014-08-19)
------------------
* comply to the new NaoNode API
* lowered getData frequency
* correct path for ros_sonar.py
* Export ros_sonar into python package
  The upper class ros_sonar.py can now be imported via 'import nao_sensors.ros_sonar'
* use the newer API when possible
* use the new NaoNode API
* comply to the newer NaoNode API
* added sonar publisher
  ros_sonar.py as common publisher for any sonar.
  reusable for different robots
  nao_sonar.py instantiats the correct sonar for nao
* rename the package from nao_camera to nao_sensors
* add octomap support
* add a dependency on nao_driver for the Python code
* Dependency on camera_info_manager_py for the python node
* do not poll the image if nobody is subscribed
* comply to REP008
* add support for 3d camera in Python
* make the camera dynamically reconfigurable in Python
* move the nao_camera.py script from nao_driver
* remove useless rosbuild artefact
* Nodelet support for Nao cameras
  Again, closely build upon camera1394 ROS driver
* Make sure we never publish more than the actual camera framerate
* Added sample calibration files for Nao cameras
  OpenCV Calibration for top and bottom cameras, at 160x120, 320x240 and 640x480.
  The actual calibration needs to be redone for each Nao, but this should be a fair
  starting point.
* Cosmetic refactoring
* Remove useless mutex
* Only publish frames when someone is listening on the topic
  This saves a lot of CPU on Nao when images are not needed.
* No easy way to go 'local'
  From a performance point of view, it would be very beneficial to be a
  NAOqi 'local module' to access the video buffer.
  It remains to be seen if we can write a ROS nodes that also happen to be a
  NAOqi local module (that would allow much better performances for the camera
  node).
* Support only RGB8 colorspace
  If BGR is useful to someone, we can re-introduce it. For now, keep
  things simple.
* Use the Nao TF frames + generate unique calibration files
  Make sure we generate calibration files different for each resolution/camera
* Added support for dynamic reconfiguration of all Nao v4 camera params
  Only reconfiguration of FPS, resolution, and source requires restart
  of the driver.
  Also, correctly unsubscribe/release from the video proxy.
* Initial working version
  This first version supports publishing images from the top
  camera on Nao. No configurability yet.
  The nodelet code has been kept, but it is not
  used/enabled at compilation yet.
* Contributors: Armin Hornung, Karsten Knese, Séverin Lemaignan, Vincent Rabaud
