

import torch
import numpy as np
import logging

torch.set_num_threads(1)

logger = logging.getLogger('hr.ros_offline_asr.silero_vad')

class SileroVad:
    def __init__(self, model, confidence, sensitivity, event=None) -> None:
        self.model = torch.jit.load(model)
        self.confidence = confidence
        self.sensitivity = sensitivity
        self.event = event

    # Converts audio to float32 np array as required by silero
    def int2float(self, sound):
        abs_max = np.abs(sound).max()
        # For the VAD  sensitivity will increase audio amplitude 
        divider = 32768 / self.sensitivity
        divider = max(abs_max, divider)
        sound   = sound.astype('float32')
        if abs_max > 0:
            sound *= 1/(divider)
        sound   = sound.squeeze()
        return sound
    
    # Accepts Int16 numpy array as sound input
    def vad(self, sound):
        sound = self.int2float(sound)
        voice_confidence = self.model(torch.from_numpy(sound), 16000).item()
        self.model.reset_states()
        if self.event and voice_confidence >= self.confidence:
            self.event.set()
        #print(voice_confidence)
        return voice_confidence >= self.confidence
    

    