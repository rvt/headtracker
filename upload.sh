#!/bin/bash

set -e 

#List mDNS devices
#platformio device list --mdns --logical | grep arduino


# ping ARILUX00AB5E97.local

# ping ARILUX00879231.local
./generateHtmlArray.sh 
#platformio run
platformio run --target upload -e wemos 
platformio device monitor --baud 115200

# connect serial IO
#platformio device monitor

#platformio test -e native
