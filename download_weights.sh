#!/bin/bash

if [ "$#" -lt "1" ]; then
    WEIGHTS="yolov3-tiny.weights"
else
    WEIGHTS=$1
fi

ROOT_DIR=$(cd $(dirname '$0') && pwd)

wget -P $ROOT_DIR/model/ \
     https://pjreddie.com/media/files/$WEIGHTS
