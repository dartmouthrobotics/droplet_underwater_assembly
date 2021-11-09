#! /bin/bash
OPEN_TIME=0.2

SLEEP_TIME=180 # seconds betwen cycle
NUM_CYCLES=10

for i in {0..${OPEN_TIME}}
do
    echo "Cycle number" $i
    rosservice call /mavros/cmd/command "{broadcast: false, command: 183, confirmation: 0, param1: 0, param2: 1765, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
    sleep $OPEN_TIME
    echo "Close"
    rosservice call /mavros/cmd/command "{broadcast: false, command: 183, confirmation: 0, param1: 0, param2: 1500, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"

done
