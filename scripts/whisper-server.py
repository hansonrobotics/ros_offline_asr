#!/usr/bin/env python3

import os
import io
import json
import base64
import time
import threading
import queue

import rospy
from std_msgs.msg import String
import websocket
from ros_offline_asr.ddr_node import DDRNode
from ros_offline_asr.speech_recorder import SpeechRecorder


class WhisperServerNode(DDRNode):
    def __init__(self):
        super(WhisperServerNode, self).__init__('/hr/perception/whisper_server')

        rospy.init_node('whisper_server')

        # Static ROS parameters
        self.ws_url = rospy.get_param('~ws_url', 'ws://10.0.0.20:8000/ws')
        self.ws_url = 'ws://192.168.0.149:8000/ws'
        self.microphone_id = rospy.get_param('~microphone', 'default')

        # Dynamic params via DDR
        WhisperServerNode.enabled = self.new_param('enabled', 'Enable VAD and streaming', True)
        WhisperServerNode.continious_listening = self.new_param('continious_listening', 'Do not pause while robot speaking', False)
        WhisperServerNode.min_confidence = self.new_param('min_confidence', 'Minimum confidence to publish text', 0.0, min=0.0, max=1.0)
        WhisperServerNode.threshold = self.new_param('threshold', 'VAD threshold', 0.5, min=0.1, max=0.99)
        WhisperServerNode.min_silence_duration_ms = self.new_param('min_silence_duration_ms', 'Min silence duration in ms', 400, min=100, max=3000)
        WhisperServerNode.speech_pad_ms = self.new_param('speech_pad_ms', 'Speech padding in ms', 200, min=0, max=1000)

        # Publishers
        self._transcript_pub = rospy.Publisher('offline_speech', String, queue_size=1)

        # Recorder
        self.sr = SpeechRecorder(
            microphone=self.microphone_id,
            threshold=self.threshold,
            min_silence_duration_ms=self.min_silence_duration_ms,
            speech_pad_ms=self.speech_pad_ms,
            sample_rate=16000,
        )

        # State
        self._paused = False
        self._ws_app = None
        self._ws_lock = threading.Lock()
        self._ws_connected = threading.Event()
        self._outbox = queue.Queue()

        # Threads
        self.ddstart()
        self._start_threads()

        # Optionally pause during robot speech
        robot_speech_event_topic = rospy.get_param('robot_speech_event_topic', '/hr/control/speech/event')
        rospy.Subscriber(robot_speech_event_topic, String, self._robot_speech_event_cb, queue_size=1)

    def _start_threads(self):
        t_rec = threading.Thread(target=self.sr.start, name='recorder')
        t_rec.daemon = True
        t_rec.start()

        t_ws = threading.Thread(target=self._ws_loop, name='ws-loop')
        t_ws.daemon = True
        t_ws.start()

        t_send = threading.Thread(target=self._send_recordings_loop, name='send-recordings')
        t_send.daemon = True
        t_send.start()

    def config_updated(self, config):
        # Apply enabled/pause
        self.sr.enabled = config.enabled and not self._paused
        # Reconfigure VAD iterator
        self.sr.reconfigure(threshold=config.threshold,
                            min_silence_duration_ms=config.min_silence_duration_ms,
                            speech_pad_ms=config.speech_pad_ms)
        return config

    # --- WebSocket management (callback-based) ---
    def _ws_loop(self):
        while not rospy.is_shutdown():
            if not self.enabled or self._paused:
                # Ensure closed when disabled/paused
                with self._ws_lock:
                    if self._ws_app is not None:
                        try:
                            self._ws_app.close()
                        except Exception:
                            pass
                        self._ws_app = None
                        self._ws_connected.clear()
                time.sleep(0.2)
                continue

            try:
                ws_app = self._build_ws_app(self.ws_url)
                with self._ws_lock:
                    self._ws_app = ws_app
                rospy.loginfo('Connecting to %s', self.ws_url)
                # ping to keepalive
                ws_app.run_forever(ping_interval=20, ping_timeout=10)
            except Exception as e:
                rospy.logwarn('WS run_forever failed: %s', e)
            finally:
                with self._ws_lock:
                    self._ws_connected.clear()
                    self._ws_app = None
                time.sleep(1.0)

    def _build_ws_app(self, url):
        return websocket.WebSocketApp(
            url,
            on_open=self._on_open,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )

    def _on_open(self, ws):
        rospy.loginfo('Connected to %s', self.ws_url)
        print("CONNECTED")
        self._ws_connected.set()

    def _on_message(self, ws, message):
        print(message)
        # message can be str (text) or bytes
        if isinstance(message, (bytes, bytearray)):
            return
        try:
            msg = json.loads(message)
        except Exception:
            return
        self._process_server_message(msg)

    def _on_error(self, ws, error):
        rospy.logwarn('WS error: %s', error)

    def _on_close(self, ws, code, reason):
        rospy.loginfo('WS closed: %s %s', code, reason)
        self._ws_connected.clear()

    def _send_recordings_loop(self):
        while not rospy.is_shutdown():
            # Block until recording available
            pcm = self.sr.recorded_queue.get()
            print("GOT Audio")
            if pcm is None:
                continue
            if not self.enabled or self._paused:
                continue
            # Wait for socket connection
            if not self._ws_connected.wait(timeout=5.0):
                continue

            # Package as WAV in-memory
            wav_bytes = self._np_float32_to_wav_bytes(pcm, sample_rate=16000, channels=1)
            print("Sending")
            message = {
                'type': 'audio',
                'audio_data': base64.b64encode(wav_bytes).decode('ascii'),
                'sample_rate': 16000,
                'channels': 1,
            }
            payload = json.dumps(message)
            with self._ws_lock:
                ws_app = self._ws_app
            try:
                if ws_app is not None and ws_app.sock and ws_app.sock.connected:
                    ws_app.send(payload)
                    print("SENT")
            except Exception as e:
                rospy.logwarn('WS send failed: %s', e)

    def _np_float32_to_wav_bytes(self, data, sample_rate=16000, channels=1):
        import numpy as np
        import struct
        data = np.asarray(data, dtype=np.float32)
        num_samples = data.shape[0]
        bits_per_sample = 32
        byte_rate = sample_rate * channels * (bits_per_sample // 8)
        block_align = channels * (bits_per_sample // 8)
        subchunk1_size = 16
        audio_format = 3  # IEEE float
        data_bytes = data.tobytes()
        data_size = len(data_bytes)
        chunk_size = 4 + (8 + subchunk1_size) + (8 + data_size)

        header = struct.pack(
            '<4sI4s4sIHHIIHH4sI',
            b'RIFF', chunk_size, b'WAVE',
            b'fmt ', subchunk1_size, audio_format, channels, sample_rate, byte_rate, block_align, bits_per_sample,
            b'data', data_size
        )
        return header + data_bytes

    def _process_server_message(self, message):
        msg_type = message.get('type')
        if msg_type == 'transcription':
            text = message.get('text', '')
            confidence = float(message.get('confidence', 1.0))
            if text and confidence >= float(self.min_confidence):
                self._transcript_pub.publish(String(data=text))
                rospy.loginfo("ws_streamer: transcription '%s'", text)
                self._console_print(f"Transcription: {text}")

    def _console_print(self, msg):
        try:
            print(msg)
        except Exception:
            pass

    def _robot_speech_event_cb(self, msg: String):
        if not self.enabled:
            return
        if msg.data:
            if msg.data.startswith('start') and not self.continious_listening:
                self._paused = True
            if msg.data.startswith('stop') and self._paused:
                self._paused = False
            if msg.data.startswith('silence') and self._paused:
                self._paused = False
        self.sr.enabled = self.enabled and not self._paused


# wave helper
import wave
class wave_open:
    def __init__(self, fileobj, mode='rb'):
        self.fileobj = fileobj
        self.mode = mode
        self.wf = None
    def __enter__(self):
        self.wf = wave.open(self.fileobj, self.mode)
        return self.wf
    def __exit__(self, exc_type, exc, tb):
        if self.wf:
            self.wf.close()


if __name__ == '__main__':
    node = WhisperServerNode()
    rospy.spin()
