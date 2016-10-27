#!/usr/bin/env python
#                                                                             
#  Copyright 2016 Aldebaran                                                   
#                                                                             
#  Licensed under the Apache License, Version 2.0 (the "License");            
#  you may not use this file except in compliance with the License.           
#  You may obtain a copy of the License at                                    
#                                                                             
#      http://www.apache.org/licenses/LICENSE-2.0                             
#                                                                             
#  Unless required by applicable law or agreed to in writing, software        
#  distributed under the License is distributed on an "AS IS" BASIS,          
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   
#  See the License for the specific language governing permissions and        
#  limitations under the License.                                             
#                                                                             
# 
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from std_srvs.srv import(
    EmptyResponse,
    Empty)
from naoqi_bridge_msgs.srv import(
    GetActiveTargetResponse,
    GetActiveTarget,
    GetAvailableModesResponse,
    GetAvailableModes,
    GetEffectorResponse,
    GetEffector,
    GetExtractorPeriodResponse,
    GetExtractorPeriod,    
    GetMaximumAccelerationResponse,
    GetMaximumAcceleration,
    GetMaximumDistanceDetectionResponse,
    GetMaximumDistanceDetection,
    GetMaximumVelocityResponse,
    GetMaximumVelocity,
    GetModeResponse,
    GetMode,
    ALTrackerGetMoveConfigResponse,
    ALTrackerGetMoveConfig,
    GetRegisteredTargetsResponse,
    GetRegisteredTargets,
    GetRelativePositionResponse,
    GetRelativePosition,
    ALTrackerGetRobotPositionResponse,
    ALTrackerGetRobotPosition,
    GetSearchFractionMaxSpeedResponse,
    GetSearchFractionMaxSpeed,
    GetSupportedTargetsResponse,
    GetSupportedTargets,
    #GetTargetCoordinatesResponse,
    #GetTargetCoordinates,
    GetTargetPositionResponse,
    GetTargetPosition,
    GetTimeOutResponse,
    GetTimeOut,
    IsActiveResponse,
    IsActive,
    IsNewTargetDetectedResponse,
    IsNewTargetDetected,
    IsSearchEnabledResponse,
    IsSearchEnabled,
    IsTargetLostResponse,
    IsTargetLost,
    LookAtResponse,
    LookAt,
    PointAtResponse,
    PointAt,
    SetEffectorResponse,
    SetEffector,
    SetExtractorPeriodResponse,
    SetExtractorPeriod,
    SetMaximumAccelerationResponse,
    SetMaximumAcceleration,
    SetMaximumDistanceDetectionResponse,
    SetMaximumDistanceDetection,
    SetMaximumVelocityResponse,
    SetMaximumVelocity,
    SetModeResponse,
    SetMode,
    SetMoveConfigResponse,
    SetMoveConfig,
    SetRelativePositionResponse,
    SetRelativePosition,
    SetSearchFractionMaxSpeedResponse,
    SetSearchFractionMaxSpeed,
    SetTimeOutResponse,
    SetTimeOut,
    TrackResponse,
    Track,
    ToggleSearchResponse,
    ToggleSearch,
    UnregisterTargetResponse,
    UnregisterTarget,
    UnregisterTargetsResponse,
    UnregisterTargets,
    RegisterRedBallResponse,
    RegisterRedBall,
    RegisterFaceResponse,
    RegisterFace,
    RegisterLandMarkResponse,
    RegisterLandMark,
    RegisterLandMarksResponse,
    RegisterLandMarks,
    RegisterPeopleResponse,
    RegisterPeople,
    RegisterSoundResponse,
    RegisterSound,
    #RegisterTargetResponse,
    #RegisterTarget,
    #SetTargetCoordinatesResponse,
    #SetTargetCoordinates,
    #TrackEventResponse,
    #TrackEvent,
)

class NaoqiTracker (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_tracker')
        self.connectNaoQi()
        self.getActiveTargetSrv = rospy.Service("get_active_target", GetActiveTarget, self.handleGetActiveTarget)
        self.getAvailableModesSrv = rospy.Service("get_available_modes", GetAvailableModes, self.handleGetAvailableModes)
        self.getEffectorSrv = rospy.Service("get_effector", GetEffector, self.handleGetEffector)
        self.getExtractorPeriodSrv = rospy.Service("get_extractor_period", GetExtractorPeriod, self.handleGetExtractorPeriod)
        self.getMaximumAccelerationSrv = rospy.Service("get_maximum_acceleration", GetMaximumAcceleration, self.handleGetMaximumAcceleration)
        self.getMaximumDistanceDetectionSrv = rospy.Service("get_maximum_distance_detection", GetMaximumDistanceDetection, self.handleGetMaximumDistanceDetection)
        self.getMaximumVelocitySrv = rospy.Service("get_maximum_velocity", GetMaximumVelocity, self.handleGetMaximumVelocity)
        self.getModeSrv = rospy.Service("get_mode", GetMode, self.handleGetMode)
        self.getMoveConfigSrv = rospy.Service("tracker_get_move_config", ALTrackerGetMoveConfig, self.handleGetMoveConfig)
        self.getRegisteredTargetsSrv = rospy.Service("get_registered_targets", GetRegisteredTargets, self.handleGetRegisteredTargets)
        self.getRelativePositionSrv = rospy.Service("get_relative_position", GetRelativePosition, self.handleGetRelativePosition)
        self.getRobotPositionSrv = rospy.Service("get_robot_position", ALTrackerGetRobotPosition, self.handleGetRobotPosition)
        self.getSearchFractionMaxSpeedSrv = rospy.Service("get_search_fraction_max_speed", GetSearchFractionMaxSpeed, self.handleGetSearchFractionMaxSpeed)
        self.getSupportedTargetsSrv = rospy.Service("get_supported_targets", GetSupportedTargets, self.handleGetSupportedTargets)
        #self.getTargetCoordinatesSrv = rospy.Service("get_target_coordinates", , self.handleGetTargetCoordinates)
        self.getTargetPositionSrv = rospy.Service("get_target_position", GetTargetPosition, self.handleGetTargetPosition)
        self.getTimeOutSrv = rospy.Service("get_time_out", GetTimeOut, self.handleGetTimeOut)
        self.initializeSrv = rospy.Service("initialize", Empty, self.handleInitialize)
        self.isNewTargetDetectedSrv = rospy.Service("is_new_target_detected", IsNewTargetDetected, self.handleIsNewTargetDetected)
        self.isSearchEnabledSrv = rospy.Service("is_search_enabled", IsSearchEnabled, self.handleIsSearchEnabled)
        self.isTargetLostSrv = rospy.Service("is_target_lost", IsTargetLost, self.handleIsTargetLost)
        self.lookAtSrv = rospy.Service("look_at", LookAt, self.handleLookAt)
        self.pointAtSrv = rospy.Service("point_at", PointAt, self.handlePointAt)
        ##self.registerTargetSrv = rospy.Service("register_target", , self.handleRegisterTarget)
        self.registerRedBallSrv = rospy.Service("register_red_ball", RegisterRedBall, self.handleRegisterRedBall)
        self.registerFaceSrv = rospy.Service("register_face", RegisterFace, self.handleRegisterFace)
        self.registerLandMarkSrv = rospy.Service("register_land_mark", RegisterLandMark, self.handleRegisterLandMark)
        self.registerLandMarksSrv = rospy.Service("register_land_marks", RegisterLandMarks, self.handleRegisterLandMarks)
        self.registerPeopleSrv = rospy.Service("register_people", RegisterPeople, self.handleRegisterPeople)
        self.registerSoundSrv = rospy.Service("register_sound", RegisterSound, self.handleRegisterSound)
        self.setEffectorSrv = rospy.Service("set_effector", SetEffector, self.handleSetEffector)
        self.setExtractorPeriodSrv = rospy.Service("set_extractor_period", SetExtractorPeriod, self.handleSetExtractorPeriod)
        self.setMaximumAccelerationSrv = rospy.Service("set_maximum_acceleration", SetMaximumAcceleration, self.handleSetMaximumAcceleration)
        self.setMaximumDistanceDetectionSrv = rospy.Service("set_maximum_distance_detection", SetMaximumDistanceDetection, self.handleSetMaximumDistanceDetection)
        self.setMaximumVelocitySrv = rospy.Service("set_maximum_velocity", SetMaximumVelocity, self.handleSetMaximumVelocity)
        self.setModeSrv = rospy.Service("set_mode", SetMode, self.handleSetMode)
        self.setMoveConfigSrv = rospy.Service("set_move_config", SetMoveConfig, self.handleSetMoveConfig)
        self.setRelativePositionSrv = rospy.Service("set_relative_position", SetRelativePosition, self.handleSetRelativePosition)
        self.setSearchFractionMaxSpeedSrv = rospy.Service("set_search_fraction_max_speed", SetSearchFractionMaxSpeed, self.handleSetSearchFractionMaxSpeed)
        #self.setTargetCoordinatesSrv = rospy.Service("set_target_coordinates", , self.handleSetTargetCoordinates)
        self.setTimeOutSrv = rospy.Service("set_time_out", SetTimeOut, self.handleSetTimeOut)
        self.stopTrackerSrv = rospy.Service("stop_tracker", Empty, self.handleStopTracker)
        self.trackSrv = rospy.Service("track", Track, self.handleTrack)
        #self.trackEventSrv = rospy.Service("track_event", , self.handleTrackEvent)
        self.toggleSearchSrv = rospy.Service("toggle_search", ToggleSearch, self.handleToggleSearch)
        self.unregisterAllTargetsSrv = rospy.Service("unregister_all_targets", Empty, self.handleUnregisterAllTargets)
        self.unregisterTargetSrv = rospy.Service("unregister_target", UnregisterTarget, self.handleUnregisterTarget)
        self.unregisterTargetsSrv = rospy.Service("unregister_targets", UnregisterTargets, self.handleUnregisterTargets)
        rospy.loginfo("naoqi_tracker is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.trackerProxy = self.get_proxy("ALTracker")
        if self.trackerProxy is None:
            exit(1)
    
    def handleGetActiveTarget(self, req):
        try:
            res = GetActiveTargetResponse()
            res.active_target = self.trackerProxy.getActiveTarget()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetAvailableModes(self, req):
        try:
            res = GetAvailableModesResponse()
            data_list = self.trackerProxy.getActiveTarget()
            if data_list != None and (len(data_list) > 0):
                for i in range(len(data_list)):
                    res.available_modes.append(data_list[i]) 
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetEffector(self, req):
        try:
            res = GetEffectorResponse()
            res.active_effector = self.trackerProxy.getEffector()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetExtractorPeriod(self, req):
        try:
            res = GetExtractorPeriodResponse()
            res.extractor_period = self.trackerProxy.getExtractorPeriod(req.target_name)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetMaximumAcceleration(self, req):
        try:
            res = GetMaximumAccelerationResponse()
            res.maximum_acceleration = self.trackerProxy.getMaximumAcceleration()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetMaximumDistanceDetection(self, req):
        try:
            res = GetMaximumDistanceDetectionResponse()
            res.maximum_distance = self.trackerProxy.getMaximumDistanceDetection()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetMaximumVelocity(self, req):
        try:
            res = GetMaximumVelocityResponse()
            res.maximum_velocity = self.trackerProxy.getMaximumVelocity()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetMode(self, req):
        try:
            res = GetModeResponse()
            res.active_mode = self.trackerProxy.getMode()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetMoveConfig(self, req):
        try:
            res = ALTrackerGetMoveConfigResponse()
            data_list = self.trackerProxy.getMoveConfig()
            if data_list is not None:
                res.move_config.max_vel_xy = (data_list[0])[1]
                res.move_config.max_vel_theta = (data_list[1])[1]
                res.move_config.max_acc_xy = (data_list[2])[1]
                res.move_config.max_acc_theta = (data_list[3])[1]
                res.move_config.max_jerk_xy = (data_list[4])[1]
                res.move_config.max_jerk_theta = (data_list[5])[1]
                res.move_config.torso_wy = (data_list[6])[1]
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetRegisteredTargets(self, req):
        try:
            res = GetRegisteredTargetsResponse()
            data_list = self.trackerProxy.getRegisteredTargets()
            if data_list != None and (len(data_list) > 0):
                for i in range(len(data_list)):
                    res.registered_targets.append(data_list[i]) 
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetRelativePosition(self, req):
        try:
            res = GetRelativePositionResponse()
            data_list = self.trackerProxy.getRelativePosition()
            res.target_position.linear.x = data_list[0]
            res.target_position.linear.y = data_list[1]
            res.target_position.linear.z = data_list[2]
            res.target_position.angular.x = data_list[3]
            res.target_position.angular.y = data_list[4]
            res.target_position.angular.z = data_list[5]
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetRobotPosition(self, req):
        try:
            res = ALTrackerGetRobotPositionResponse()
            data_list = self.trackerProxy.getRobotPosition()
            res.robot_position.linear.x = data_list[0]
            res.robot_position.linear.y = data_list[1]
            res.robot_position.linear.z = data_list[2]
            res.robot_position.angular.x = data_list[3]
            res.robot_position.angular.y = data_list[4]
            res.robot_position.angular.z = data_list[5]
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetSearchFractionMaxSpeed(self, req):
        try:
            res = GetSearchFractionMaxSpeedResponse()
            res.fraction_max_speed = self.trackerProxy.getSearchFractionMaxSpeed()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetSupportedTargets(self, req):
        try:
            res = GetSupportedTargetsResponse()
            res.supported_target_names = self.trackerProxy.getSupportedTargets()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

   # def handleGetTargetCoordinates(self, req):
   #      try:
   #          res = GetTargetCoordinatesResponse()
   #          data_list = self.trackerProxy.getTargetCoordinates()
   #          res.first_object_coordinate.x = data_list[0][0]
   #          res.first_object_coordinate.y = data_list[0][1]
   #          res.first_object_coordinate.z = data_list[0][2]
   #          res.second_object_coordinate.x = data_list[1][0]
   #          res.second_object_coordinate.y = data_list[1][1]
   #          res.second_object_coordinate.z = data_list[1][2]
   #          return res
   #      except RuntimeError, e:
   #          rospy.logerr("Exception caught:\n%s", e)
   #          return None

    def handleGetTargetPosition(self, req):
        try:
            res = GetTargetPositionResponse()
            data_list = self.trackerProxy.getTargetPosition(req.frame.data)
            res.target_position.x = data_list[0]
            res.target_position.y = data_list[1]
            res.target_position.z = data_list[2]
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetTimeOut(self, req):
        try:
            res = GetTimeOutResponse()
            res.timeout = self.trackerProxy.getTimeOut()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleInitialize(self, req):
        try:
            res = EmptyResponse()
            self.trackerProxy.initialize()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsActive(self, req):
        try:
            res = IsActiveResponse()
            res.status = self.trackerProxy.isActive()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsNewTargetDetected(self, req):
        try:
            res = IsNewTargetDetectedResponse()
            res.status = self.trackerProxy.isNewTargetDetected()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsSearchEnabled(self, req):
        try:
            res = IsSearchEnabledResponse()
            res.status = self.trackerProxy.isSearchEnabled()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleIsTargetLost(self, req):
        try:
            res = IsTargetLostResponse()
            res.status = self.trackerProxy.isTargetLost()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleLookAt(self, req):
        res = LookAtResponse()
        res.success = False
        try:
            position = []
            position[0] = req.position.x
            position[1] = req.position.y
            position[2] = req.position.z
            self.trackerProxy.lookAt(position, req.frame, req.fraction_max_frame, req.use_whole_body)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handlePointAt(self, req):
        res = PointAtResponse()
        res.success = False
        try:
            if req.arm.data == 0:
                arm = "LArm"
            elif req.arm.data == 1:
                arm = "RArm"
            else:
                arm = "Arms"
            position = []
            position[0] = req.position.x
            position[1] = req.position.y
            position[2] = req.position.z
            self.trackerProxy.pointAt(arm, position, req.frame, req.fraction_max_frame)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleRegisterRedBall(self, req):
        res = RegisterRedBallResponse()
        res.success = False
        try:
            self.trackerProxy.registerTarget("RedBall", req.diameter)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleRegisterFace(self, req):
        res = RegisterFaceResponse()
        res.success = False
        try:
            self.trackerProxy.registerTarget("Face", req.width_of_face)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleRegisterLandMark(self, req):
        res = RegisterLandMarkResponse()
        res.success = False
        try:
            data_list = [req.land_mark.size, req.land_mark.land_mark_id]
            self.trackerProxy.registerTarget("LandMark", data_list)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleRegisterLandMarks(self, req):
        res = RegisterLandMarksResponse()
        res.success = False
        try:
            data_list = []
            for i in range (len(req.land_marks)):
                data_list.append ([((req.land_marks)[i]).size, ((req.land_marks)[i]).land_mark_id])
            self.trackerProxy.registerTarget("LandMarks", data_list)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleRegisterPeople(self, req):
        res = RegisterPeopleResponse()
        res.success = False
        try:
            self.trackerProxy.registerTarget("People", req.people_id)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleRegisterSound(self, req):
        res = RegisterSoundResponse()
        res.success = False
        try:
            self.trackerProxy.registerTarget("Sound", [req.distance, req.confidence])
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetEffector(self, req):
        res = SetEffectorResponse()
        res.success = False
        try:
            self.trackerProxy.setEffector(req.effector)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetExtractorPeriod(self, req):
        res = SetExtractorPeriodResponse()
        res.success = False
        try:
            self.trackerProxy.setExtractorPeriod(req.target_name, req.period)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMaximumAcceleration(self, req):
        res = SetMaximumAccelerationResponse()
        res.success = False
        try:
            self.trackerProxy.setMaximumAcceleration(req.max_acceleration)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMaximumDistanceDetection(self, req):
        res = SetMaximumDistanceDetectionResponse()
        res.success = False
        try:
            self.trackerProxy.setMaximumDistanceDetection(req.max_distance)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMaximumVelocity(self, req):
        res = SetMaximumVelocityResponse()
        res.success = False
        try:
            self.trackerProxy.setMaximumVelocity(req.max_velocity)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMode(self, req):
        res = SetModeResponse()
        res.success = False
        try:
            self.trackerProxy.setMode(req.mode)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetMoveConfig(self, req):
        res = SetMoveConfigResponse()
        res.success = False
        try:
            move_config = []
            move_config.append(["MaxVelXY", req.move_config.max_vel_xy])
            move_config.append(["MaxVelTheta", req.move_config.max_vel_theta])
            move_config.append(["MaxAccXY", req.move_config.max_acc_xy])
            move_config.append(["MaxAccTheta", req.move_config.max_acc_theta])
            move_config.append(["MaxJerkXY", req.move_config.max_jerk_xy])
            move_config.append(["MaxJerkTheta", req.move_config.max_jerk_theta])
            move_config.append(["TorsoWy", req.move_config.torso_wy])
            self.trackerProxy.setMoveConfig(move_config)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetRelativePosition(self, req):
        res = SetRelativePositionResponse()
        res.success = False
        try:
            target_position = []
            target_position.append(req.target.linear.x)
            target_position.append(req.target.linear.y)
            target_position.append(req.target.linear.z)
            target_position.append(req.target.angular.x)
            target_position.append(req.target.angular.y)
            target_position.append(req.target.angular.z)
            self.trackerProxy.setRelativePosition(target_position)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    # def handleSetTargetCoordinates(self, req):
    #     try:
    #         pass
    #         return res
    #     except RuntimeError, e:
    #         rospy.logerr("Exception caught:\n%s", e)
    #         return None

    def handleSetSearchFractionMaxSpeed(self, req):
        res = SetSearchFractionMaxSpeedResponse()
        res.success = False
        try:
            self.trackerProxy.setSearchFractionMaxSpeed(req.fraction_max_speed)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetTimeOut(self, req):
        try:
            res = SetTimeOutResponse()
            res.timeout = self.trackerProxy.setTimeOut(req.timeout)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleStopTracker(self, req):
        try:
            res = EmptyResponse()
            self.trackerProxy.stopTracker()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleTrack(self, req):
        res = TrackResponse()
        res.success = False
        try:
            self.trackerProxy.track(req.target_name)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleToggleSearch(self, req):
        res = ToggleSearchResponse()
        res.success = False
        try:
            self.trackerProxy.toggleSearch(req.search_on)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleUnregisterAllTargets(self, req):
        try:
            res = EmptyResponse()
            self.trackerProxy.unregisterAllTargets()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleUnregisterTarget(self, req):
        res = UnregisterTargetResponse()
        res.success = False
        try:
            self.trackerProxy.unregisterTarget(req.target_name)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleUnregisterTargets(self, req):
        res = UnregisterTargetsResponse()
        res.success = False
        try:
            self.trackerProxy.unregisterTargets(req.target_names)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def run(self):
        while self.is_looping():
            try:
                pass
            
            except RuntimeError, e:
                print "Error accessing ALMemory, exiting...\n"
                print e
                rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    tracker = NaoqiTracker()
    tracker.start()
    rospy.spin()
