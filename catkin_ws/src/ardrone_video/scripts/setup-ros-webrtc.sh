#!/bin/bash

cd ../../ros_webrtc/test/provision
ansible-galaxy install -r requirements.yml
ansible-playbook -i 'localhost,' -c local -K dev.yml

