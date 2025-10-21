import numpy as np
import sounddevice as sd
import threading
import queue
import time
import wave
import torch
from silero_vad import VADIterator
from silero_vad import load_silero_vad

class SpeechRecorder:
    def __init__(self,
                 microphone,
                 sample_rate=16000,
                 threshold: float = 0.5,
                 min_silence_duration_ms: int = 400,
                 speech_pad_ms: int = 200,
                 buffer_duration: float = 0.5):
        self.device = microphone
        self.enabled = True
        self.sample_rate = int(sample_rate)
        self.threshold = float(threshold)
        self.min_silence_duration_ms = int(min_silence_duration_ms)
        self.speech_pad_ms = int(speech_pad_ms)
        self.buffer_duration = float(buffer_duration)

        # Queues
        self.recorded_queue = queue.Queue()
        self._chunk_q = queue.Queue()

        # Ring buffer to keep pre-roll audio (float32)
        self._ring = np.zeros(shape=(0,), dtype=np.float32)
        self.buffer_lock = threading.Lock()
        self.recording = False
        self.recording_data = np.zeros(shape=(0,), dtype=np.float32)

        # Load Silero VAD (onnx) and create iterator
        self.vad = load_silero_vad(onnx=True)
        self.vad_iterator = VADIterator(
            self.vad,
            threshold=self.threshold,
            sampling_rate=self.sample_rate,
            min_silence_duration_ms=self.min_silence_duration_ms,
            speech_pad_ms=self.speech_pad_ms,
        )

        # Streaming counters
        self._chunk_samples = 512 if self.sample_rate == 16000 else 256
        self._samples_seen = 0

    def reconfigure(self, threshold: float = None, min_silence_duration_ms: int = None, speech_pad_ms: int = None):
        updated = False
        if threshold is not None and float(threshold) != self.threshold:
            self.threshold = float(threshold)
            updated = True
        if min_silence_duration_ms is not None and int(min_silence_duration_ms) != self.min_silence_duration_ms:
            self.min_silence_duration_ms = int(min_silence_duration_ms)
            updated = True
        if speech_pad_ms is not None and int(speech_pad_ms) != self.speech_pad_ms:
            self.speech_pad_ms = int(speech_pad_ms)
            updated = True
        if updated:
            # Recreate iterator with new params
            self.vad_iterator = VADIterator(
                self.vad,
                threshold=self.threshold,
                sampling_rate=self.sample_rate,
                min_silence_duration_ms=self.min_silence_duration_ms,
                speech_pad_ms=self.speech_pad_ms,
            )
            self.vad_iterator.reset_states()

    def record_callback(self, indata, frames, time, status):
        if not self.enabled:
            return
        # Expect float32 input in [-1, 1] shape (frames, 1)
        # Push mono chunk to processing queue
        try:
            chunk = indata[:, 0].astype(np.float32, copy=True)
        except Exception:
            # Fallback if shape unexpected
            chunk = np.asarray(indata).astype(np.float32).reshape(-1)
        # Enforce fixed window size expected by model
        if len(chunk) != self._chunk_samples:
            # If driver delivers different frames, reslice/pad to window
            if len(chunk) > self._chunk_samples:
                chunk = chunk[:self._chunk_samples]
            else:
                pad = np.zeros(self._chunk_samples - len(chunk), dtype=np.float32)
                chunk = np.concatenate([chunk, pad])
        self._chunk_q.put(chunk)

    def vad_thread_func(self):
        buffer_len = int(self.buffer_duration * self.sample_rate)
        while True:
            if not self.enabled:
                time.sleep(0.01)
                continue

            try:
                chunk_f32 = self._chunk_q.get(timeout=0.1)
            except queue.Empty:
                continue

            # Prepare tensors for VADIterator
            x = torch.from_numpy(chunk_f32).unsqueeze(0)  # [1, N]
            event = self.vad_iterator(x)

            # If start detected, include pre-roll from ring based on reported start index
            if event and 'start' in event and not self.recording:
                print("Start")
                start_idx = int(event['start'])
                pre_needed = max(self._samples_seen - start_idx, 0)
                if pre_needed > 0:
                    pre_needed = int(min(pre_needed, len(self._ring)))
                    if pre_needed > 0:
                        self.recording_data = np.concatenate((self.recording_data, self._ring[-pre_needed:]))
                self.recording = True

            # While recording, append current chunk
            if self.recording:
                self.recording_data = np.concatenate((self.recording_data, chunk_f32))

            # If end detected, finalize utterance
            if event and 'end' in event and self.recording:
                print("end")
                # Compute and print simple level metrics for this utterance
                if len(self.recording_data) > 0:
                    dur_s = len(self.recording_data) / float(self.sample_rate)
                    mean_abs = float(np.mean(np.abs(self.recording_data)))
                    rms = float(np.sqrt(np.mean(self.recording_data ** 2)))
                    dbfs = 20.0 * np.log10(max(rms, 1e-12))
                    print(f"Utterance: {dur_s:.2f}s, mean|x|={mean_abs:.4f}, rms={rms:.4f}, dBFS={dbfs:.1f}")
                    print(self.vad_iterator.threshold)
                # Push only if we have at least 0.2s of audio
                if len(self.recording_data) > int(0.2 * self.sample_rate):
                    self.recorded_queue.put(self.recording_data.copy())
                self.recording = False
                self.recording_data = np.zeros(shape=(0,), dtype=np.float32)
                self.vad_iterator.reset_states()

            # Update ring buffer and counters
            with self.buffer_lock:
                self._ring = np.concatenate((self._ring, chunk_f32))
                if len(self._ring) > buffer_len:
                    self._ring = self._ring[-buffer_len:]
            self._samples_seen += self._chunk_samples

    def save_thread_func(self):
        file_num = 1
        while True:
            recording_data = self.recorded_queue.get()
            # Write IEEE float32 WAV for debugging
            data = np.asarray(recording_data, dtype=np.float32)
            header = self._build_float_wav_header(len(data), self.sample_rate, 1)
            with open(f'recording_{file_num}.wav', 'wb') as f:
                f.write(header)
                f.write(data.tobytes())
            file_num += 1

    def pause(self, ignore_recorded=False):
        with self.buffer_lock:
            self._ring = np.zeros(shape=(0,), dtype=np.float32)
        if not ignore_recorded:
            # At least 0.2 seconds recording needed
            if self.recording and len(self.recording_data) > 0.2 * self.sample_rate:
                self.recorded_queue.put(self.recording_data)
                self.recording_data = np.zeros(shape=(0,), dtype=np.float32)
        # Reset iterator state between sessions
        self.vad_iterator.reset_states()

            

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
                with sd.InputStream(callback=self.record_callback,
                                    device=self.device,
                                    channels=1,
                                    dtype='float32',
                                    blocksize=self._chunk_samples,
                                    samplerate=self.sample_rate):
                    while self.enabled:
                        time.sleep(0.1)
                    # Reset buffers
                    self.pause(ignore_recorded=True)

    @staticmethod
    def _build_float_wav_header(num_samples: int, sample_rate: int, channels: int) -> bytes:
        import struct
        bits_per_sample = 32
        byte_rate = sample_rate * channels * (bits_per_sample // 8)
        block_align = channels * (bits_per_sample // 8)
        subchunk1_size = 16  # fmt chunk size
        audio_format = 3     # IEEE float
        data_size = num_samples * (bits_per_sample // 8)
        chunk_size = 4 + (8 + subchunk1_size) + (8 + data_size)

        header = struct.pack(
            '<4sI4s4sIHHIIHH4sI',
            b'RIFF', chunk_size, b'WAVE',
            b'fmt ', subchunk1_size, audio_format, channels, sample_rate, byte_rate, block_align, bits_per_sample,
            b'data', data_size
        )
        return header


if __name__ == "__main__":
    # Simple smoke test: print recorded chunk lengths
    rec = SpeechRecorder(microphone='default')
    t = threading.Thread(target=rec.start)
    t.daemon = True
    t.start()
    while True:
        data = rec.recorded_queue.get()
        print(f"Recorded chunk length: {len(data)} samples")
