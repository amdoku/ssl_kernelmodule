#!/bin/bash

pushd .
cd /opt/poky/2.6.4/sysroots/cortexa9hf-neon-poky-linux-gnueabi/usr/src/kernel
sudo bash -c "source /opt/poky/2.6.4/environment-setup-cortexa9hf-neon-poky-linux-gnueabi && make silentoldconfig scripts"
popd

