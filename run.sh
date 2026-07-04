#!/bin/bash
SRC_DIR=$(dirname $0)
SRC_DIR=$(cd "$SRC_DIR" && pwd)
if ! cd "$SRC_DIR"
then
	echo "Failed to cd in $SRC_DIR"
	exit 1
fi
if ! pgrep pigpiod 
then
	if ! ./pigpiod
	then
		echo "Failed to run pigpiod"
		exit 1
	fi
fi
source venv/bin/activate
python3 main.py
pkill -9 pigpiod
