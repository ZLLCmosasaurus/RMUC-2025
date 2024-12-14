#!/bin/bash

JlinkScript="./jlink_script.jlink"

if [ -f $JlinkScript ]; then
    rm $JlinkScript
fi
touch $JlinkScript
echo h > $JlinkScript
echo loadfile $1 >> $JlinkScript
echo r >> $JlinkScript
echo g >> $JlinkScript
echo exit >> $JlinkScript

jlink -device STM32F427II -autoconnect 1 -if SWD -speed 4000 -CommanderScript $JlinkScript
