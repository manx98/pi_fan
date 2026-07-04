#!/bin/bash
SRC_DIR=$(dirname $0)
cd "$SRC_DIR"
SRC_DIR=$(pwd)
rm -rf venv
if ! python3 -m venv venv
then
	echo "Failed to create python venv"
	exit 1
fi

if ! source venv/bin/activate
then
	echo "Failed active venv"
	exit 1
fi

if ! pip3 install -r requirements.txt -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
then
	echo "Failed to install requirements"
	exit 1
fi
cp pi_fan.service /etc/systemd/system
SRC_DIR_ESCAPED=$(printf '%s\n' "$SRC_DIR" | sed 's/[&]/\\&/g')
chmod +x /etc/systemd/system/pi_fan.service
sed -i "s|{SRC_DIR}|${SRC_DIR_ESCAPED}|g" /etc/systemd/system/pi_fan.service
systemctl enable pi_fan.service
systemctl start pi_fan.service
