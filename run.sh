#!/bin/bash
set -u

SRC_DIR=$(dirname "$0")
SRC_DIR=$(cd "$SRC_DIR" && pwd)
if ! cd "$SRC_DIR"
then
	echo "Failed to cd in $SRC_DIR"
	exit 1
fi

if ! source venv/bin/activate
then
	echo "Failed to activate venv"
	exit 1
fi

exec python3 main.py
