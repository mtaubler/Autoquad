#!/bin/bash

set -e
./install-python.sh
sudo python get-pip.py
./install-node.sh
./install-kms.sh
./install-ansible.sh
./setup-ros-webrtc.sh

echo yay!
