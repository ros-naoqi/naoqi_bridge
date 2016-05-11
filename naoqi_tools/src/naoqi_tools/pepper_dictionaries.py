############################
######### PEPPER ###########
############################
Pepper_offsets = {
    # Base_footprint
    'BaseFootprintOffsetX': 0.0,
    'BaseFootprintOffsetY': 0.0,
    'BaseFootprintOffsetZ': -0.334,
    # KneePitch to Wheel : 0.264 + WheelRadius 0.07 = 0.334
    'BaseFootprintRotX': 0.0,
    'BaseFootprintRotY': 0.0,
    'BaseFootprintRotZ': 0.0,
}

Pepper_links = {
    'HeadYaw_link': 'Neck',
    'HeadPitch_link': 'Head',
    'HipRoll_link': 'Hip',
    'HipPitch_link': 'Pelvis',
    'KneePitch_link': 'Tibia',
    'Torso_link': 'torso',
    'LShoulderPitch_link': 'LShoulder',
    'RShoulderPitch_link': 'RShoulder',
    'LShoulderRoll_link': 'LBicep',
    'RShoulderRoll_link': 'RBicep',
    'LElbowYaw_link': 'LElbow',
    'RElbowYaw_link': 'RElbow',
    'LElbowRoll_link': 'LForeArm',
    'RElbowRoll_link': 'RForeArm',
    'LWristYaw_link': 'l_wrist',
    'RWristYaw_link': 'r_wrist',
    'LHand_actuator_frame': 'l_gripper',
    'RHand_actuator_frame': 'r_gripper',

    # SENSORS
    'Accelerometer_sensor': 'ImuTorsoAccelerometer_frame',
    'AccelerometerBase_sensor': 'ImuBaseAccelerometer_frame',
    'GyrometerBase_sensor': 'ImuBaseGyrometer_frame',
    'Gyrometer_sensor': 'ImuTorsoGyrometer_frame',
    'CameraBottom_sensor': 'CameraBottom_frame',
    'CameraDepth_sensor': 'CameraDepth_frame',
    'CameraTop_sensor': 'CameraTop_frame',
    'ChestBoard/Button_sensor': 'ChestButton_frame',
    'Head/Touch/Front_sensor': 'HeadTouchFront_frame',
    'Head/Touch/Middle_sensor': 'HeadTouchMiddle_frame',
    'Head/Touch/Rear_sensor': 'HeadTouchRear_frame',
    'LHand/Touch/Back_sensor': 'LHandTouchBack_frame',
    'RHand/Touch/Back_sensor': 'RHandTouchBack_frame',
    'LaserSensor/Front_sensor': 'SurroundingFrontLaser_frame',
    'LaserSensor/Left_sensor': 'SurroundingLeftLaser_frame',
    'LaserSensor/Right_sensor': 'SurroundingRightLaser_frame',
    'LaserSensor/Shovel_sensor': 'ShovelLaser_frame',
    'LaserSensor/VerticalLeft_sensor': 'VerticalLeftLaser_frame',
    'LaserSensor/VerticalRight_sensor': 'VerticalRightLaser_frame',
    'Sonar/Back_sensor': 'SonarBack_frame',
    'Sonar/Front_sensor': 'SonarFront_frame',
    'SpeakerLeft_sensor': 'LSpeaker_frame',
    'SpeakerRight_sensor': 'RSpeaker_frame',
    'Tablet_sensor': 'Tablet_frame',
    'Bumper/Back_sensor': 'BumperB_frame',
    'Bumper/FrontLeft_sensor': 'BumperFL_frame',
    'Bumper/FrontRight_sensor': 'BumperFR_frame',
}
