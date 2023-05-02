import numpy as np
import sounddevice as sd
import threading
import queue
import time
import wave

class SpeechRecorder:
    def __init__(self, vad, microphone,  buffer_duration=0.5, silence_duration=0.5, chunk_size=0.1, vad_window=0.2, sample_rate=16000):
        self.device = microphone
        self.enabled = True
        self.vad = vad
        self.buffer_duration = buffer_duration
        self.silence_duration = silence_duration
        self.chunk_size = chunk_size
        self.vad_window = vad_window
        self.sample_rate = sample_rate

        self.recorded_queue = queue.Queue()

        self.audio_buffer = np.zeros(shape=(0,), dtype=np.int16)
        self.buffer_lock = threading.Lock()
        self.recording = False
        self.recording_data = np.zeros(shape=(0,), dtype=np.int16)
        self.last_vad_time = 0

    def record_callback(self, indata, frames, time, status):
        if not self.enabled:
            return
        with self.buffer_lock:
            self.audio_buffer = np.concatenate((self.audio_buffer, indata[:, 0].astype(np.int16)))
            buffer_len = int(self.buffer_duration * self.sample_rate)
            if len(self.audio_buffer) > buffer_len:
                self.audio_buffer = self.audio_buffer[-buffer_len:]

    def vad_thread_func(self):
        while True:
            if self.enabled and len(self.audio_buffer) >= int(self.vad_window * self.sample_rate):
                with self.buffer_lock:
                    # This chunk needed for VAD
                    vad_chunk = self.audio_buffer[:int(self.vad_window * self.sample_rate)]
                    # Rolling window 
                    chunk = self.audio_buffer[:int(self.chunk_size * self.sample_rate)]
                    self.audio_buffer = self.audio_buffer[int(self.chunk_size * self.sample_rate):]
                if self.vad(vad_chunk):
                    self.last_vad_time = time.time()
                    if not self.recording:
                        self.recording = True
                    self.recording_data = np.concatenate((self.recording_data, chunk))
                elif self.recording and time.time() - self.last_vad_time > self.silence_duration:
                    self.recording = False
                    self.recorded_queue.put(self.recording_data)
                    self.recording_data = np.zeros(shape=(0,), dtype=np.int16)
                elif self.recording:
                    self.recording_data = np.concatenate((self.recording_data, chunk))
                else:
                    # IF not recording, keep the buffer at max buffer_druteation
                    self.recording_data = np.concatenate((self.recording_data, chunk))
                    self.recording_data = self.recording_data[-int(self.buffer_duration * self.sample_rate):]
            time.sleep(self.chunk_size/2.0)

    def save_thread_func(self):
        file_num = 1
        while True:
            recording_data = self.recorded_queue.get()
            with wave.open(f'recording_{file_num}.wav', 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self.sample_rate)
                wav_file.writeframes(recording_data)
            file_num += 1

    def pause(self, ignore_recorded=False):
        with self.buffer_lock:
            self.audio_buffer = np.zeros(shape=(0,), dtype=np.int16)
        if not ignore_recorded:
            # At least 0.2 seconds recording needed
            if self.recording and len(self.recording_data) > 0.2 * self.sample_rate:
                self.recorded_queue.put(self.recording_data)
                self.recording_data = np.zeros(shape=(0,), dtype=np.int16)

            

    def start(self):
        self.vad_thread = threading.Thread(target=self.vad_thread_func)
        self.vad_thread.daemon = True
        #self.save_thread = threading.Thread(target=self.save_thread_func)
        self.vad_thread.start()
        #self.save_thread.start()
        while True:
            time.sleep(0.01)
            # Start audio recording
            if self.enabled:
                with sd.InputStream(callback=self.record_callback,device=self.device, channels=1, dtype=np.int16, samplerate=self.sample_rate):
                    while self.enabled:
                        time.sleep(0.1)
                    # Rest buffers
                    self.pause(ignore_recorded=True)




if __name__ == "__main__":
    from .silero_vad import SileroVad
    Vad = SileroVad("/home/hr/workspace/hrsdk_configs/models/silero_vad/silero_vad.jit", 0.8)
    recorder = SpeechRecorder(vad=Vad.vad)
    recorder.start()


