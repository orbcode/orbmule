source /usr/local/share/orbcode/gdbtrace.init

#!orbtrace -T 4 -e vtref,on -p vtref,3.3

!gnome-terminal --hide-menubar -- /usr/bin/openocd -f /usr/local/share/orbcode/muleboard.ocd
target remote localhost:3333
file orbdemo.elf
monitor reset halt
load
set mem inaccessible-by-default off


#prepareSWO 160000000 40000000 1 1
#enableSTM32SWO 4
enableSTM32TRACE 4


dwtSamplePC 1
dwtSyncTap 1
dwtPostTap 1
dwtPostInit 2
dwtPostReset 15
dwtCycEna 1
dwtTraceException 0

ITMId 1
ITMGTSFreq 1
ITMTSPrescale 0
ITMTXEna 1
ITMSYNCEna 0
ITMEna 1
ITMTSEna 1
ITMTER 0 0xFFFFFFFF
ITMTPR 0xFFFFFFFF
