#!/usr/bin/env bash

if [ "$#" -ne 6 ]; then
    echo "Usage: ${0} data_dir p_id object_name play_rate"
    echo "Got ${#} arguments"
    exit -1
fi

DATA_DIR=$1
P_ID=$2
OBJECT_NAME=$3
PLAY_RATE=$4

BAG_FILENAME=$(<$DATA_DIR/$P_ID/$OBJECT_NAME/recording.txt)

rosbag play --clock --keep-alive -d 2 -r $PLAY_RATE \
$DATA_DIR/$P_ID/$OBJECT_NAME/$BAG_FILENAME
