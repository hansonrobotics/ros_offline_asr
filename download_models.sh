#!/usr/bin/env bash
wget https://alphacephei.com/kaldi/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
mv vosk-model-small-en-us-0.15 models/en-US
rm vosk-model-small-en-us-0.15.zip

wget https://alphacephei.com/vosk/models/vosk-model-small-cn-0.3.zip
unzip vosk-model-small-cn-0.3.zip
mv vosk-model-small-cn-0.3 models/cmn-Hans-CN
rm vosk-model-small-cn-0.3.zip

wget https://alphacephei.com/vosk/models/vosk-model-small-de-0.15.zip
unzip vosk-model-small-de-0.15.zip
mv vosk-model-small-de-0.15 models/de-DE
rm vosk-model-small-de-0.15.zip