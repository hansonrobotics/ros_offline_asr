#!/usr/bin/env python3
#
# Copyright (c) 2025 Hanson Robotics.
#
# This file is part of Hanson AI.
# See https://www.hansonrobotics.com/hanson-ai for further info.
#
# Licensed under the MIT License.
# See LICENSE file in the project root for full license information.
#

import os
import logging
import rospy
from threading import Event, Thread
from queue import Empty, Queue
from ros_offline_asr.ddr_node import DDRNode
from ros_offline_asr.silero_vad import SileroVad
from ros_offline_asr.speech_recorder import SpeechRecorder
from ros_offline_asr.whisper_speech_recognizer import WhisperSpeechRecognizer

from std_msgs.msg import String

logger = logging.getLogger('hr.ros_offline_asr.ros_offline')

class WhipseNode(DDRNode):
    
    def __init__(self):
        # Super
        super(WhipseNode, self).__init__('/hr/perception/whisper_asr')

        rospy.init_node('whisper_asr')
        current_dir = os.path.dirname(os.path.realpath(__file__))
        # Default model directory
        self.model_dir= rospy.get_param('~model_dir', '/home/hr/workspace/hrsdk_configs/models')
        self.microphone_id = rospy.get_param('~microphone', 'default')
        # Paused (while robot speaking)
        self.paused = False
        WhipseNode.enabled = self.new_param("enabled", "Enable VAD and STT", True)
        WhipseNode.continuous = self.new_param("continuous", "Dont pause while robot speaking", True)
        WhipseNode.model = self.new_param("model", "Whisper model", "base", edit_method=self.enum(["tiny", "base", "small"]))
        WhipseNode.language = self.new_param("language", "Language code (use zh for Chinese)", "en")
        WhipseNode.vad_confidence = self.new_param("vad_confidence", "Voice Activity detection confidence)", 0.9, min=0.5, max=0.99)
        WhipseNode.vad_sensitivity = self.new_param("vad_sensitivity", "Voice Activity sensitivity)", 0.5, min=0.1, max=3.0)
        
        # Make silvero VAD, 
        # TODO publish vad messages
        self.vad_event = Event()
        silero_model = os.path.join(self.model_dir,'silero_vad','silero_vad.onnx')
        self.sileroVad = SileroVad(silero_model, self.vad_confidence, self.vad_sensitivity, event=self.vad_event)
        # Make speech recorder
        self.speech_recorder = SpeechRecorder(self.sileroVad.vad, self.microphone_id)
        # Make whisper recognizer
        self.results = Queue()
        whisper_models = os.path.join(self.model_dir,'faster-whisper')
        self.whisper = WhisperSpeechRecognizer(whisper_models, self.speech_recorder.recorded_queue, self.results)
        # Start audio recording
        self.speech_pub = rospy.Publisher('offline_speech', String, queue_size=1)
        self.vad_pub = rospy.Publisher('vad', String, queue_size=1)
        
        self.whisper.start()
        self.ddstart()
        self.recorder_thread = Thread(target=self.speech_recorder.start)
        self.recorder_thread.daemon = True
        self.recorder_thread.start()


        robot_speech_event_topic = rospy.get_param('robot_speech_event_topic', '/hr/control/speech/event')
        rospy.Subscriber(robot_speech_event_topic, String, self.robot_speech_event_cb, queue_size=1)
        self.paused = False
        # Publisher thread
        self.results_thread = Thread(target=self.publish_results)
        self.results_thread.daemon = True
        self.results_thread.start()

        # Publisher thread
        self.results_thread = Thread(target=self.publish_vad)
        self.results_thread.daemon = True
        self.results_thread.start()

    def config_updated(self, config):
        self.sileroVad.sensitivity = config.vad_sensitivity
        self.sileroVad.confidence = config.vad_confidence
        self.speech_recorder.enabled = config.enabled and not self.paused
        self.whisper.change_model(config.model)
        self.whisper.change_language(config.language)

        return config

    def robot_speech_event_cb(self, msg):
        if not self.enabled:
            return
        """ Used in continuous listening mode """
        if msg.data:
            if msg.data.startswith('start') and not self.continuous:
                # Reset speech recognition then TTS starts (restarting after TTS finish might be too late).
                self.paused = True
            if msg.data.startswith('stop') and self.paused:
                self.paused = False
            # wait for silence message if available
            if msg.data.startswith('silence') and self.paused:
                self.paused = False
        self.speech_recorder.enabled = self.enabled and not self.paused

   

    def publish_results(self):
        while not rospy.is_shutdown():
            try:
                res = self.results.get(timeout=1.0)
                self.publish_result(res)
            except Empty:
                continue


    def publish_vad(self):
        while not rospy.is_shutdown():
            res = self.vad_event.wait(timeout=1.0)
            if (res is True):
                self.vad_pub.publish('vad')
                self.vad_event.clear()


    def publish_result(self, result):
        self.speech_pub.publish(result)

if __name__ == '__main__':
    asr = WhipseNode()
    rospy.spin()


