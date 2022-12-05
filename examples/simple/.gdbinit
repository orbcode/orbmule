# For this example, have 'blackmagic' running already. No command line params
# are needed.
!orbtrace -e vtref,on -p vtref,3.3 -T m
file ofiles/simple.elf
target extended-remote localhost:2000
set mem inaccessible-by-default off
monitor swd
att 1
load

source /usr/local/bin/orbcode//gdbtrace.init
enableSTM32SWO 4
prepareSWO 16000000 8000000 0 1
dwtSamplePC 1
dwtSyncTap 3
dwtPostTap 1
dwtPostInit 1
dwtPostReset 10
dwtCycEna 1

ITMTXEna 1
ITMEna 1

