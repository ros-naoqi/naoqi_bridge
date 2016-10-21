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
from std_msgs.msg import (
    String,
    Bool)
from naoqi_bridge_msgs.msg import (
    WordRecognized,
    WordRecognizedAndGrammar)
from std_srvs.srv import (
     EmptyResponse,
     Empty)
from naoqi_bridge_msgs.srv import (
    GetAvailableLanguagesResponse,
    GetAvailableLanguages,
    GetLanguageResponse,
    GetLanguage,
    SetLanguageResponse,
    SetLanguage,
    GetSpeechRecognitionParameterResponse,
    GetSpeechRecognitionParameter,
    SetSpeechRecognitionParameterResponse,
    SetSpeechRecognitionParameter,
    GetSpeechRecognitionParameterResponse,
    GetSpeechRecognitionParameter,
    GetAudioExpressionResponse,
    GetAudioExpression,
    SetAudioExpressionResponse,
    SetAudioExpression,
    SetVisualExpressionResponse,
    SetVisualExpression,
    SetVocabularyResponse,
    SetVocabulary,
    CompileBNFToLCFResponse,
    CompileBNFToLCF,
    AddContextResponse,
    AddContext,
    RemoveContextResponse,
    RemoveContext,
    SaveContextSetResponse,
    SaveContextSet,
    LoadContextSetResponse,
    LoadContextSet,
    EraseContextSetResponse,
    EraseContextSet,
    ActivateRuleResponse,
    ActivateRule,
    DeactivateRuleResponse,
    DeactivateRule,
    ActivateAllRulesResponse,
    ActivateAllRules,
    DeactivateAllRulesResponse,
    DeactivateAllRules,
    AddWordListToSlotResponse,
    AddWordListToSlot,
    RemoveWordListFromSlotResponse,
    RemoveWordListFromSlot,
    GetRulesResponse,
    GetRules,
    PauseResponse,
    Pause)

class NaoqiSpeechRecognition (NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'naoqi_speechRecognition')
        self.connectNaoQi()
        self.wordRecognizedPub = rospy.Publisher("word_recognized", WordRecognized, queue_size=10)
        self.wordRecognizedAndGrammarPub = rospy.Publisher("word_recognized_and_grammar", WordRecognizedAndGrammar, queue_size=10)
        self.speechDetectedPub = rospy.Publisher("speech_detected", Bool, queue_size=10)
        self.isRunningPub = rospy.Publisher("ALSpeechRecognition/IsRunning", Bool, queue_size=10)
        self.statusPub = rospy.Publisher("ALSpeechRecognition/Status", String, queue_size=10)
        self.activeListeningPub = rospy.Publisher("ALSpeechRecognition/ActiveListening", Bool, queue_size=10)
        #services
        self.getAvailableLanguagesSrv = rospy.Service("get_available_languages", GetAvailableLanguages, self.handleGetAvailableLanguages)
        self.getLanguageSrv = rospy.Service("get_language", GetLanguage, self.handleGetLanguage)
        self.setLanguageSrv = rospy.Service("set_language", SetLanguage, self.handleSetLanguage)
        self.getSpeechRecognitionParameterSrv = rospy.Service("get_speech_recognition_parameter", GetSpeechRecognitionParameter, self.handleGetSpeechRecognitionParameter)
        self.setSpeechRecognitionParameterSrv = rospy.Service("set_speech_recognition_parameter", SetSpeechRecognitionParameter, self.handleSetSpeechRecognitionParameter)
        self.getAudioExpressionSrv = rospy.Service("get_audio_expression", GetAudioExpression, self.handleGetAudioExpression)
        self.setAudioExpressionSrv = rospy.Service("set_audio_expression", SetAudioExpression, self.handleSetAudioExpression)
        self.setVisualExpressionSrv = rospy.Service("set_visual_expression", SetVisualExpression, self.handleSetVisualExpression)
        self.setVocabularySrv = rospy.Service("set_vocabulary", SetVocabulary, self.handleSetVocabulary)
        self.compileSrv = rospy.Service("compile_bnf_to_lcf", CompileBNFToLCF, self.handleCompile)
        self.addContextSrv = rospy.Service("add_context", AddContext, self.handleAddContext)
        self.removeContextSrv = rospy.Service("remove_context", RemoveContext, self.handleRemoveContext)
        self.removeAllContextSrv = rospy.Service("remove_all_context", Empty, self.handleRemoveAllContext)
        self.saveContextSetSrv = rospy.Service("save_context_set", SaveContextSet, self.handleSaveContextSet)
        self.loadContextSetSrv = rospy.Service("load_context_set", LoadContextSet, self.handleLoadContextSet)
        self.eraseContextSetSrv = rospy.Service("erase_context_set", EraseContextSet, self.handleEraseContextSet)
        self.activateRuleSrv = rospy.Service("activate_rule", ActivateRule, self.handleActivateRule)
        self.deactivateRuleSrv = rospy.Service("deactivate_rule", DeactivateRule, self.handleDeactivateRule)
        self.activateAllRulesSrv = rospy.Service("activate_all_rules", ActivateAllRules, self.handleActivateAllRules)
        self.deactivateAllRulesSrv = rospy.Service("deactivate_all_rules", DeactivateAllRules, self.handleDeactivateAllRules)
        self.addWordListToSlotSrv = rospy.Service("add_word_list_to_slot", AddWordListToSlot, self.handleAddWordListToSlot)
        self.removeWordListFromSlotSrv = rospy.Service("remove_word_list_from_slot", RemoveWordListFromSlot, self.handleRemoveWordListFromSlot)
        self.getRulesSrv = rospy.Service("get_rules", GetRules, self.handleGetRules)
        self.pauseSrv = rospy.Service("pause", Pause, self.handlePause)
        self.subscribeSrv = rospy.Service("start_speech_recognition", Empty, self.handleStartSpeechRecognition)
        self.unsubscribeSrv = rospy.Service("stop_speech_recognition", Empty, self.handleStopSpeechRecognition)
        
        rospy.loginfo("naoqi_speech_recognition is initialized")
       
    def connectNaoQi(self):
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)
        self.memProxy = self.get_proxy("ALMemory")
        self.srProxy = self.get_proxy("ALSpeechRecognition")
        if self.memProxy is None or self.srProxy is None:
            exit(1)

    def handleGetAvailableLanguages(self, req):
        try:
            res = GetAvailableLanguagesResponse()
            res.languages = self.srProxy.getAvailableLanguages() 
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleGetLanguage(self, req):
        try:
            res = GetLanguageResponse()
            res.language = self.srProxy.getLanguage() 
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetLanguage(self, req):
        res = SetLanguageResponse()
        res.success = False
        try:
            self.srProxy.setLanguage(req.language)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleGetSpeechRecognitionParameter(self, req):
        try:
            res = GetSpeechRecognitionParameterResponse()
            res.value = self.srProxy.getParameter(req.parameter)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetSpeechRecognitionParameter(self, req):
        res = SetSpeechRecognitionParameterResponse()
        res.success = False
        try:
            self.srProxy.setParameter(req.parameter, req.value)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
    def handleGetAudioExpression(self, req):        
        try:
            res = GetAudioExpressionResponse()
            res.setOrNot = self.srProxy.getAudioExpression()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleSetAudioExpression(self, req):        
        res = SetAudioExpressionResponse()
        res.success = False
        try:
            res.success = self.srProxy.setAudioExpression(req.setOrNot)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleSetVisualExpression(self, req):
        res = SetVisualExpressionResponse()
        res.success = False
        try:
            res.success = self.srProxy.setVisualExpression(req.setOrNot)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
    def handleSetVocabulary(self, req):
        res = SetVocabularyResponse()
        res.success = False
        try:
            res.success = self.srProxy.setVocabulary(req.vocabulary, req.enableWordSpotting)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
    def handleCompile(self, req):
        res = CompileBNFToLCFResponse()
        res.success = False
        try:
            res.success = self.srProxy.compile(req.pathToInputBNFFile, req.pathToOutputLCFFile, req.language)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
    def handleAddContext(self, req):
        res = AddContextResponse()
        res.success = False
        try:
            res.success = self.srProxy.addContext(req.pathToOutputLCFFile, req.contextName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleRemoveContext(self, req):
        res = RemoveContextResponse()
        res.success = False
        try:
            res.success = self.srProxy.removeContext(req.contextName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleRemoveAllContext(self, req):
        try:
            res = EmptyResponse()
            self.srProxy.removeAllContext()
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None
    
    def handleSaveContextSet(self, req):
        res = SaveContextSetResponse()
        res.success = False
        try:
            self.srProxy.saveContextSet(req.saveName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
    def handleLoadContextSet(self, req):
        res = LoadContextSetResponse()
        res.success = False
        try:
            self.srProxy.loadContextSet(req.saveName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleEraseContextSet(self, req):
        res = EraseContextSetResponse()
        res.success = False
        try:
            self.srProxy.eraseContextSet(req.saveName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleActivateRule(self, req):
        res = ActivateRuleResponse()
        res.success = False
        try:
            self.srProxy.activateRule(req.contextName, req.ruleName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
        
    def handleDeactivateRule(self, req):
        res = DeactivateRuleResponse()
        res.success = False
        try:
            self.srProxy.deactivateRule(req.contextName, req.ruleName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleActivateAllRules(self, req):
        res = ActivateAllRulesResponse()
        res.success = False
        try:
            self.srProxy.activateAllRules(req.contextName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleDeactivateAllRules(self, req):
        res = DeactivateAllRulesResponse()
        res.success = False
        try:
            self.srProxy.deactivateAllRules(req.contextName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res

    def handleAddWordListToSlot(self, req):
        res = AddWordListToSlotResponse()
        res.success = False
        try:
            self.srProxy.addWordListToSlot(req.contextName, req.slotName, req.wordList)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
            
    def handleRemoveWordListFromSlot(self, req):
        res = RemoveWordListFromSlotResponse()
        res.success = False
        try:
            self.srProxy.removeWordListFromSlot(req.contextName, req.slotName)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
            
    def handleGetRules(self, req):
        try:
            res = GetRulesResponse()
            res.rules = self.srProxy.getRules(req.contextName, req.typeName)
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handlePause(self, req):
        res = PauseResponse()
        res.success = False
        try:
            self.srProxy.pause(req.isPaused)
            res.success = True
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return res
    
    def handleStartSpeechRecognition(self, req):
        try:
            res = EmptyResponse()
            self.srProxy.subscribe("speech_recognition")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def handleStopSpeechRecognition(self, req):
        try:
            res = EmptyResponse()
            self.srProxy.unsubscribe("speech_recognition")
            return res
        except RuntimeError, e:
            rospy.logerr("Exception caught:\n%s", e)
            return None

    def run(self):
        while self.is_looping():
            try:
                # WordRecognized
                word_recognized_msg = WordRecognized()
                word_recognized_data = self.memProxy.getData("WordRecognized")
                if (len (word_recognized_data) > 0) :
                    for i in range ((len(word_recognized_data))/ 2):
                        word_recognized_msg.words.append(word_recognized_data[2 * i])
                        word_recognized_msg.confidence_values.append(word_recognized_data[2 * i + 1])
                    self.wordRecognizedPub.publish(word_recognized_msg)

                # WordRecognizedAndGrammar
                word_recognized_and_grammar_msg = WordRecognizedAndGrammar()
                word_recognized_and_grammar_data = self.memProxy.getData("WordRecognizedAndGrammar")
                if (len (word_recognized_and_grammar_data) > 0) :
                    for i in range ((len(word_recognized_data)) / 3):
                        word_recognized_msg.words.append(word_recognized_data[3 * i])
                        word_recognized_msg.confidence_values.append(word_recognized_data[3 * i + 1])
                        word_recognized_msg.grammars.append(word_recognized_data[3 * i + 2])
                    self.wordRecognizedAndGrammarPub.publish(word_recognized_and_grammar_msg)

                # SpeechDetected
                speech_detected_msg = Bool()
                speech_detected_msg.data = self.memProxy.getData("SpeechDetected")
                self.speechDetectedPub.publish(speech_detected_msg)

                # ALSpeechRecognition/IsRunning
                is_running_msg = Bool()
                is_running_msg.data = self.memProxy.getData("ALSpeechRecognition/IsRunning")
                self.isRunningPub.publish(is_running_msg)

                # ALSpeechRecognition/Status
                status_msg = String()
                status_msg.data = self.memProxy.getData("ALSpeechRecognition/Status")
                self.statusPub.publish(status_msg)

                # ALSpeechRecognition/ActiveListening
                active_listening_msg = Bool()
                active_listening_msg.data = self.memProxy.getData("ALSpeechRecognition/ActiveListening")
                self.activeListeningPub.publish(active_listening_msg)

            except RuntimeError, e:
                 print "Error accessing ALMemory, exiting...\n"
                 print e
                 rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    speech_recognition = NaoqiSpeechRecognition()
    speech_recognition.start()
    rospy.spin()
