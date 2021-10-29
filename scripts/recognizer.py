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
import json
import rospy
import vosk
import sounddevice
from queue import Queue
from threading import Event, Thread, Lock
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
                self.samplerate = int(device_info['default_samplerate'])
        except Exception as e:
            logger.error(f"Wasnt able to get device information {e}")
            exit(1)
        self.enabled = False
        self.enabled_e = Event()
        # Event to make sure Model is loaded
        self.model_load_e = Event()
        self.audio_q = Queue()
        self.model_lock = Lock()
        self.speech_pub = rospy.Publisher('offline_speech', String, queue_size=1)
        self.interim_speech_pub = rospy.Publisher('offline_interim_speech', String, queue_size=1)
        self.last_partial = ""
        self.current_lang = ""
        self.model = None
        self.reconfigure_srv = Server(OfflineAsrConfig, self.config_cb)
        self.run()


    def reload_model(self):
        model_thread = Thread(target=self.load_model)
        model_thread.daemon = True
        model_thread.start()



    def load_model(self):
        # Make sure models are not reloaded simultaniously
        with self.model_lock:
            self.model = None
            self.model_load_e.clear()
            model_path = os.path.join(self.model_dir, self.current_lang)
            if not os.path.exists(model_path):
                logger.error(f"Model was not found at {model_path}")
                # Make sure to update configuration so its clear there was an error
                self.reconfigure_srv.update_configuration({'enable':False})
                return
            try:
                # Load model 
                m  = vosk.Model(model_path=model_path)
                # Once model is loaded update reference
                self.model = m
                self.model_load_e.set()
            except Exception as e:
                self.reconfigure_srv.update_configuration({'enable':False})
                logger.error(f"Model load failed with exception {e}")

    def config_cb(self, config, lvl=None):
        if config.language != self.current_lang:
            self.current_lang = config.language
            self.reload_model()
        if config.enable != self.enabled:
            if config.enable:
                self.start_recognition()
            else:
                self.stop_recognition()
        return config
    
    def start_recognition(self):
        # Model should be loaded at this point
        self.enabled = True
        if not self.enabled_e.is_set():
            self.enabled_e.set()

    def stop_recognition(self):
        self.enabled = False
        self.enabled_e.clear()

    def microphone_callback(self, indata, frames, time, status):
        """ Callback to read microphone data """
        if status:
            logger.warn(f"Microphone data returned {status}")
        # Dtata storage
        self.audio_q.put(bytes(indata))

    def run(self):
        while not rospy.is_shutdown():
            # Block until SR is enabled, dont block forever to gracefully exit if killed
            self.enabled_e.wait(timeout=1.0)
            if not self.enabled:
                continue
            # Block until SR is enabled, dont block forever to gracefully exit if killed
            self.model_load_e.wait(timeout=1.0)
            if not self.model_load_e.is_set():
                continue
            with sounddevice.RawInputStream(samplerate=self.samplerate, blocksize = 8000, device=self.microphone_id,
                                            dtype='int16', channels=1, callback=self.microphone_callback):

                rec = vosk.KaldiRecognizer(self.model, self.samplerate)
                #rec.SetMaxAlternatives(3) - no need alternatives for now
                # Require model to work
                while self.enabled and self.model is not None and not rospy.is_shutdown():
                    data = self.audio_q.get()
                    if rec.AcceptWaveform(data):
                        self.publish_result(json.loads(rec.Result()))
                    else:
                        self.publish_interim_result(json.loads(rec.PartialResult()))

    def publish_result(self, result):
        if result['text'] != "":
            self.speech_pub.publish(String(result['text']))
    
    def publish_interim_result(self, result):
        # Publish partial result if changed and if not empty
        if result['partial'] != "" and self.last_partial != result['partial']:
            self.interim_speech_pub.publish(String(result['partial']))
        self.last_partial = result['partial']   
            
if __name__ == '__main__':
    asr = Recognizer()










    
        

