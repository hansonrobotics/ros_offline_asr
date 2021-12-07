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
import vosk
from queue import Queue
from threading import Event, Thread, Lock

logger = logging.getLogger('hr.ros_offline_asr.offline_recognizer')

class OfflineSpeechRecognizer(object):
    """ Provides an recognition service for specific language"""
    def __init__(self, model_dir, sample_rate, results: Queue = None):
        self.model_dir= model_dir
        self.sample_rate = sample_rate
        self.enabled = False
        # Event when SR is enabled
        self.model_load_e = Event()
        # Aduio queue
        self.audio_q = Queue()
        # Resuklts queue
        if results is None:
            self.results = Queue()
        else:
            self.results = results
        self.model_lock = Lock()
        self.last_partial = ""
        self.current_lang = ""
        self.model = None
        self.running = False
        # Running Thread
        self.runner = None

    def reload_model(self):
        model_thread = Thread(target=self.load_model)
        model_thread.daemon = True
        model_thread.start()

    def load_model(self):
        # Make sure models are not reloaded simultaniously
        with self.model_lock:
            try:
                self.stop()
                self.model = None
                self.model_load_e.clear()
                model_path = os.path.join(self.model_dir, self.current_lang)
                if not os.path.exists(model_path):
                    logger.error(f"Model was not found at {model_path}")
                    return
                try:
                    # Load model 
                    print("Start Loading")
                    m  = vosk.Model(model_path=model_path)
                    print("Finish Loading")
                    # Once model is loaded update reference
                    self.model = m
                    # Starts after model is reloaded
                    self.start()
                except Exception as e:
                    logger.error(f"Model load failed with exception {e}")
            finally:
                # Model finisshed loading regardless of any errors, so it wont block the thread
                self.model_load_e.set()


    def change_language(self, lang):
        if lang != self.current_lang:
            self.current_lang = lang
            self.reload_model()


    def start(self):
        # Waits for previous thread to finish (shouldnt be the case in normal operation)
        if self.running:
            self.enabled = False
            self.runner.join()
        self.enabled = True
        self.runner = Thread(target=self.run)
        self.runner.daemon = True
        self.runner.start()


    def stop(self):
        self.enabled = False
    
    def audio_recv(self, audio):
        self.audio_q.put(audio)

    def run(self):
        # Prevent running multiple threads
        if self.running:
            return
        try:
            self.running = True
            while not self.model_load_e.is_set() and self.enabled:
                self.model_load_e.wait(timeout=1.0)
            
            rec = vosk.KaldiRecognizer(self.model, self.sample_rate)
            rec.SetMaxAlternatives(5)
            while self.enabled and self.model is not None:
                data = self.audio_q.get()
                if rec.AcceptWaveform(data):
                    self.publish_result(json.loads(rec.Result()))
                else:
                    self.publish_interim_result(json.loads(rec.PartialResult()))
        finally:
            self.running = False

    def publish_result(self, result):
        results = result['alternatives']
        for r in results:
            # If one of alternatives are no speech, ignore as likely it will be inacurate
            if r['text'] == '':
                return
        self.results.put(results[0])
    
    def publish_interim_result(self, result):
        # Publish partial result if changed and if not empty
        if result['partial'] != "" and self.last_partial != result['partial']:
            self.results.put(result)
        self.last_partial = result['partial']
        pass










    
        

