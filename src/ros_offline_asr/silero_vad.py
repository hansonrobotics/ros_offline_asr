#
# Copyright (c) 2025 Hanson Robotics.
#
# This file is part of Hanson AI.
# See https://www.hansonrobotics.com/hanson-ai for further info.
#
# Licensed under the MIT License.
# See LICENSE file in the project root for full license information.
#


import torch
import numpy as np
import logging
import numpy as np
import onnxruntime

torch.set_num_threads(1)

logger = logging.getLogger('hr.ros_offline_asr.silero_vad')

class OnnxWrapper():

    def __init__(self, path, force_onnx_cpu=False):


        opts = onnxruntime.SessionOptions()
        opts.inter_op_num_threads = 1
        opts.intra_op_num_threads = 1

        if force_onnx_cpu and 'CPUExecutionProvider' in onnxruntime.get_available_providers():
            self.session = onnxruntime.InferenceSession(path, providers=['CPUExecutionProvider'], sess_options=opts)
        else:
            self.session = onnxruntime.InferenceSession(path, sess_options=opts)

        self.reset_states()
        self.sample_rates = [8000, 16000]

    def _validate_input(self, x, sr: int):
        if x.dim() == 1:
            x = x.unsqueeze(0)
        if x.dim() > 2:
            raise ValueError(f"Too many dimensions for input audio chunk {x.dim()}")

        if sr != 16000 and (sr % 16000 == 0):
            step = sr // 16000
            x = x[:,::step]
            sr = 16000

        if sr not in self.sample_rates:
            raise ValueError(f"Supported sampling rates: {self.sample_rates} (or multiply of 16000)")

        if sr / x.shape[1] > 31.25:
            raise ValueError("Input audio chunk is too short")

        return x, sr

    def reset_states(self, batch_size=1):
        self._h = np.zeros((2, batch_size, 64)).astype('float32')
        self._c = np.zeros((2, batch_size, 64)).astype('float32')
        self._last_sr = 0
        self._last_batch_size = 0

    def __call__(self, x, sr: int):

        x, sr = self._validate_input(x, sr)
        batch_size = x.shape[0]

        if not self._last_batch_size:
            self.reset_states(batch_size)
        if (self._last_sr) and (self._last_sr != sr):
            self.reset_states(batch_size)
        if (self._last_batch_size) and (self._last_batch_size != batch_size):
            self.reset_states(batch_size)

        if sr in [8000, 16000]:
            ort_inputs = {'input': x.numpy(), 'h': self._h, 'c': self._c, 'sr': np.array(sr, dtype='int64')}
            ort_outs = self.session.run(None, ort_inputs)
            out, self._h, self._c = ort_outs
        else:
            raise ValueError()

        self._last_sr = sr
        self._last_batch_size = batch_size
        # Return VAD confidence
        return out[0][0]

    def audio_forward(self, x, sr: int, num_samples: int = 512):
        outs = []
        x, sr = self._validate_input(x, sr)

        if x.shape[1] % num_samples:
            pad_num = num_samples - (x.shape[1] % num_samples)
            x = torch.nn.functional.pad(x, (0, pad_num), 'constant', value=0.0)

        self.reset_states(x.shape[0])
        for i in range(0, x.shape[1], num_samples):
            wavs_batch = x[:, i:i+num_samples]
            out_chunk = self.__call__(wavs_batch, sr)
            outs.append(out_chunk)

        stacked = torch.cat(outs, dim=1)
        return stacked.cpu()
    

class SileroVad:
    def __init__(self, model, confidence, sensitivity, event=None) -> None:
        self.model = OnnxWrapper(model, True)
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
        voice_confidence = self.model(torch.from_numpy(sound), 16000)
        if self.event and voice_confidence >= self.confidence:
            self.event.set()
        #print(voice_confidence)
        return voice_confidence >= self.confidence
    

    