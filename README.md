# ROS Offline Speech Recognition

Offline speech recognition based on [Vosk](https://alphacephei.com/vosk/)

## System requirements:

* Ubuntu 20.04
* ROS Noetic

For other systems, the ROS node needs to be run as Python3 node.

## Installation

Install python requirements:

```bash
pip3 install -r requirements.txt
```

Download models

```bash
cd ros_offline_asr
./download_models.sh
```

## Configuration

These are the following parameters that can be passed to ROS node:

```yaml
models_dir:     /path/to/models # By default its ./models
microphone_id:  "default"       # Microphone identifier for sounddevice library
sample_rate:    ""              # Sample rate, by deefault it will use device default sample rate
```

For microphone selection see: [sounddevice.query_devices](https://python-sounddevice.readthedocs.io/en/0.3.12/api.html#sounddevice.query_devices)

