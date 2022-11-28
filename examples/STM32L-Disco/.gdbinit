file ofiles/simple.elf
target extended-remote localhost:3333
set mem inaccessible-by-default off
load

source /usr/local/share/orbcode/gdbtrace.init
dwtSamplePC 1
dwtSyncTap 3
dwtPostTap 1
dwtPostInit 1
dwtPostReset 10
dwtTraceException 1
dwtCycEna 1

ITMTXEna 1
ITMEna 1

