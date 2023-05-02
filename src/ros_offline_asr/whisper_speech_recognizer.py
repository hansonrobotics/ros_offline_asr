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
from faster_whisper import WhisperModel
from queue import Queue
from threading import Event, Thread, Lock
import numpy as np 

logger = logging.getLogger('hr.ros_offline_asr.whsiper')

class WhisperSpeechRecognizer(object):
    """ Provides an recognition service for specific language"""
    # model name should be base small or tiny
    def __init__(self, model_dir, recordings: Queue, results: Queue, model_size='base', language='en'):
        self.recordings = recordings
        self.model_dir= model_dir
        self.model_size = model_size
        self.current_lang = language
        self.model_name = self.resolve_model(model_size, self.current_lang)
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
        self.model = None
        # Running Thread
        self.runner = None
        self.reload_model()


    def resolve_model(self, model, lang):
        if lang == 'en':
            return f"{model}.{'en'}"
        return  model

    def reload_model(self):
        model_thread = Thread(target=self.load_model)
        model_thread.daemon = True
        model_thread.start()

    def load_model(self):
        # Make sure models are not reloaded simultaniously
        with self.model_lock:
            try:
                self.model = None
                model_path = os.path.join(self.model_dir, self.model_name)
                if not os.path.exists(model_path):
                    logger.error(f"Model was not found at {model_path}")
                    return
                try:
                    # Load model 
                    m = WhisperModel(model_path, device='cpu', compute_type="float32")
                    # Once model is loaded update reference
                    self.model = m
                    # Starts after model is reloaded
                except Exception as e:
                    logger.error(f"Model load failed with exception {e}")
            finally:
                # Model finisshed loading regardless of any errors, so it wont block the thread
                self.model_load_e.set()


    def change_language(self, lang):
        lang = lang[:2].lower()
        if lang != self.current_lang:
            self.current_lang = lang
            if self.resolve_model(self.model_size, self.current_lang) != self.model_name:
                self.model_name = self.resolve_model(self.model_size, self.current_lang)
                self.reload_model()


    def change_model(self, model_size):
        self.model_size = model_size
        if self.resolve_model(self.model_size, self.current_lang) != self.model_name:
            self.model_name = self.resolve_model(self.model_size, self.current_lang)
            self.reload_model()

    # Converts audio to float32 np array as required by silero
    def int2float(self, sound):
        abs_max = np.abs(sound).max()
        sound   = sound.astype('float32')
        if abs_max > 0:
            sound *= 1/32768   
            #sound *= 1/abs_max
        sound   = sound.squeeze()
        return sound
    
    # Accepts Int16 numpy 
    def start(self):
        self.enabled = True
        self.runner = Thread(target=self.run)
        self.runner.daemon = True
        self.runner.start()

    def run(self):
        while True:
            recording  = self.recordings.get()
            recording = self.int2float(recording)
            # Model needs to be loaded before process audio
            with self.model_lock:
                segments, info = self.model.transcribe(recording, beam_size=5)
                #segments, info = self.model.transcribe("recording_1.wav", beam_size=5)
                
                # Actual recognition from generator
                sentences = list(s.text for s in segments)
                #text result  
                result = " ".join(sentences).strip()
                if len(result):
                    self.results.put(result)


if __name__ == "__main__":
    from silero_vad import SileroVad
    from speech_recorder import SpeechRecorder
    from queue import Queue
    sileroVad = SileroVad("/home/hr/workspace/hrsdk_configs/models/silero_vad/silero_vad.jit", 0.8)
    recorder = SpeechRecorder(sileroVad.vad)
    results = Queue()
    recognizer = WhisperSpeechRecognizer('/home/hr/workspace/hrsdk_configs/models/faster-whisper', recorder.recorded_queue, results)
    recognizer.start()
    recorder.start()
    while True:
        print(results.get())



    
        

