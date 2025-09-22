#!/bin/bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2af2", ATTRS{idProduct}=="1002", MODE="0666"' | sudo tee /etc/udev/rules.d/99-cis-scm-camera.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

