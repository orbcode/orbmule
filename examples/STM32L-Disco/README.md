This is the readme for the simple demo project, adapted for the STM32L-Disco board.  It should work with most other STM32 boards of similar vintage as ST have always been pretty good at family compatability...in fairness, if you change the memory addresses in the link file, it should work with most M3 and above, the only interrupt we use is SysTick.

Before starting this, you will need to install the [orbuculum](https://github.com/orbcode/orbuculum) tools. Follow the instructions at the link to to that.

Now, To use the demo, connect your Disco board to your PC and start the openocd server;

```
$ openocd -f stm32ldiscovery.cfg
Open On-Chip Debugger 0.11.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
Info : Listening on port 3443 for armv7m_trace connections
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 300 kHz
Info : STLINK V2J40S0 (API v2) VID:PID 0483:3748
Info : Target voltage: 2.910517
Info : stm32l1.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : starting gdb server for stm32l1.cpu on 3333
Info : Listening on port 3333 for gdb connections
Info : accepting 'armv7m_trace' connection on tcp/3443
Info : accepting 'armv7m_trace' connection on tcp/3443
```

Build the application (note that you might need to change the path to the build tools in the Makefile!);

```
$ make
 Compiling src/itm_messages.c
 Compiling src/system_stm32l1xx.c
 Compiling src/main.c
 Compiling system/startup_ARMCM3.c
   text	   data	    bss	    dec	    hex	filename
   1271	     32	  10008	  11311	   2c2f	ofiles/simple.elf
$
```

Before starting the application, open up orbcat in another window and configure it to report messages from the target;

```
$ orbcat -c0,"%c" -c1,"%c" -c2,"Iterations in 10 secs=%d\r\n"
```

Now you can start the application which will automatically download to the target board using the included .gdbinit file;

```
$ arm-none-eabi-gdb -q
main () at src/main.c:104
104	            while (1);
Loading section .text, size 0x4f7 lma 0x8000000
Loading section .data, size 0x20 lma 0x80004f8
Start address 0x08000344, load size 1303
Transfer rate: 2 KB/sec, 651 bytes/write.
(gdb) continue
Continuing.
```

...the orbcat window will now spring to life with various information about the run;

```
Simple Example running
Sieve run ends
Sieve run starts
Sieve run ends

<SNIP>

Sieve run starts
Sieve run ends
Sieve run starts
Sieve run ends
Iterations in 10 secs=331
```

You can mess with the enabled channels in the application via the `ITM_ChannelEnable` ca;;s and the build parameters (e.g. `DEBUG`) in the Makefile to see what effect they have.

The target is also configured (via the `.gdbinit` file) to output program samples. You can report those via the `orbtop` application in another window;

```
$ orbtop -l -e ofiles/simple.elf -E -c 10

 24.40%      393 _sieve::52
 23.72%      382 _sieve::54
 21.73%      350 _sieve
 10.37%      167 _sieve::45
  7.39%      119 _sieve::58
  5.21%       84 _sieve::55
  2.23%       36 _sieve::43
  1.86%       30 _sieve::60
  1.11%       18 _sieve::51
  0.74%       12 _sieve::49
-----------------
 98.76%     1591 of  1610  Samples


 Exception         |   Count  |  MaxD | TotalTicks  |   %   |  AveTicks  |  minTicks  |  maxTicks  |  maxWall 
-------------------+----------+-------+-------------+-------+------------+------------+------------+----------
 15 (SysTick)      |      926 |     1 |          0  |  -nan |          0 |         0  |          0 |         0
```

Note that due to the limited capacity of the channel, timestamps are turned off, so you won't see the extended information about exception performance. You can mess with the parameters in `.gdbinit` to switch that information on, and you might want to switch the board to use the HSE(Bypass) crystal too in order to get better performance on the link. It's also worth noting that if you switch optimisation on then you'll lose access to line number information...anyone who has ever tried to step through optimised code will be awae that it's virtually impossible to map source to object in an optimised image.

Good luck.
