file ofiles/firmware.elf

# Power up the probe and target. This is assuming ORBTrace
# Setup for in-probe Manchester TPIU decode at 56Mbps
# --------------------------------------------------------
!orbtrace -p vtref,3.3 -e vtref,on -TM -a 56000000

# ...If using pyocd or openocd
# ----------------------------
!gnome-terminal --hide-menubar -- pyocd gdb -t stm32f429xg -f 20000000
target extended-remote localhost:3333

# ...If using BMDA
# ----------------
#!gnome-terminal --hide-menubar -- blackmagic
#target extended-remote localhost:2000
#mon swd
#attach 1

##########################################################
set mem inaccessible-by-default off
source /usr/local/share/orbcode/gdbtrace.init
stopETM
load

# For SWO
# -------
enableSTM32SWO
#          CPU Speed bitrate  T M
prepareSWO 168000000 56000000 1 1

# For parallel trace
# ------------------
#enableSTM32TRACE 4

##################################################
# Setup whatever output specifications you want
##################################################
dwtSamplePC 0
dwtSyncTap 3
dwtPostTap 0
dwtPostInit 1
dwtPostReset 15
dwtCycEna 1
dwtTraceException 0

ITMId 1
ITMGTSFreq 0
ITMTSPrescale 3
ITMTXEna 1
ITMSYNCEna 1
ITMEna 1

# For tracing
# -----------
#startETM 0 1

ITMTER 0 0xFFFFFFFF
ITMTPR 0xFFFFFFFF