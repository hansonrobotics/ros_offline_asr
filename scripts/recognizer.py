#!/usr/bin/env python3
#
# Copyright (c) 2021 Hanson Robotics.
#
# This file is part of Hanson AI.
# See https://www.hansonrobotics.com/hanson-ai for further info.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
import logging
import rospy
import time
import sounddevice
from ros_offline_asr.offline_speech_recognizer import OfflineSpeechRecognizer
from queue import Empty
from threading import Event, Thread
from std_msgs.msg import String
from ros_offline_asr.cfg import OfflineAsrConfig
from dynamic_reconfigure.server import Server


logger = logging.getLogger('hr.ros_offline_asr.ros_offline')

class Recognizer(object):
    
    def __init__(self):
        rospy.init_node('offline_asr')
        current_dir = os.path.dirname(os.path.realpath(__file__))
        # Default model directory
        model_dir = os.path.realpath(os.path.join(current_dir, '..','models'))
        self.model_dir= rospy.get_param('~model_dir', model_dir)
        # By default microphone is set to ALSA default, so can be changed in audio settings    
        self.microphone_id = rospy.get_param('~microphone', 'default')
        # Could use the device default sample rate by default
        self.sample_rate = rospy.get_param('~sampel_rate', None)
        try:
            device_info = sounddevice.query_devices(self.microphone_id, 'input')
            # soundfile expects an int, sounddevice provides a float:
            if self.sample_rate is None:
                self.sample_rate = int(device_info['default_samplerate'])
        except Exception as e:
            logger.error(f"Wasnt able to get device information {e}")
            exit(1)
        self.enabled = False
        self.enabled_e = Event()
        self.recognizer = OfflineSpeechRecognizer(model_dir=self.model_dir, sample_rate=self.sample_rate)

        # Event to make sure Model is loaded
        self.speech_pub = rospy.Publisher('offline_speech', String, queue_size=1)
        self.interim_speech_pub = rospy.Publisher('offline_interim_speech', String, queue_size=1)
        self.reconfigure_srv = Server(OfflineAsrConfig, self.config_cb)
        # Streams audio to recognizer
        self.audio_thread = Thread(target=self.capture_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        # Publishes results
        self.results_thread = Thread(target=self.publish_results)
        self.results_thread.daemon = True
        self.results_thread.start()
    
    def config_cb(self, config, lvl=None):
        # Set language
        self.recognizer.change_language(config.language)
        if config.enable != self.enabled:
            if config.enable:
                self.start_recognition()
            else:
                self.stop_recognition()
        return config
    

    def publish_results(self):
        while not rospy.is_shutdown():
            try:
                res = self.recognizer.results.get(timeout=1.0)
                self.publish_result(res)
            except Empty:
                continue


    def start_recognition(self):
        # Model should be loaded at this point
        self.enabled = True
        if not self.enabled_e.is_set():
            self.enabled_e.set()
        self.recognizer.start()

    def stop_recognition(self):
        self.enabled = False
        self.enabled_e.clear()
        self.recognizer.stop()

    def microphone_callback(self, indata, frames, time, status):
        """ Callback to read microphone data """
        if status:
            logger.warn(f"Microphone data returned {status}")
        # Put audio for recognition
        self.recognizer.audio_recv(bytes(indata))

    def capture_audio(self):
        while not rospy.is_shutdown():
            # Block until SR is enabled, dont block forever to gracefully exit if killed
            self.enabled_e.wait(timeout=1.0)
            if not self.enabled:
                continue
            with sounddevice.RawInputStream(samplerate=self.sample_rate, blocksize = 8000, device=self.microphone_id,
                                            dtype='int16', channels=1, callback=self.microphone_callback):
                while self.enabled and not rospy.is_shutdown():
                    # Callback is used to get audio data, so can do nothing here
                    time.sleep(0.1)

    def publish_result(self, result):
        if result.get('text', False):
            self.speech_pub.publish(result['text'])
        if result.get('partial', False):
            self.interim_speech_pub.publish(result['partial'])

if __name__ == '__main__':
    asr = Recognizer()
    rospy.spin()










    
        

