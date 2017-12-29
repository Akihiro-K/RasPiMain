#!/bin/bash

# run multimarkerserver in the background and save standard output to stdout.log
# run raspimain in the foreground
(cd ../MarkerServer/MultiMarkerServer/build/; ./multimarkerserver > ../output_data/stdout.log 2>&1) & (cd RasPiMain/build; sleep 1s; ./main)
