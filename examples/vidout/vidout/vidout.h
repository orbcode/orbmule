#ifndef	_VIDEO_H_
#define	_VIDEO_H_

#if defined(BLUEPILL)
#include "stm32f10x.h"
#elif defined(MULEF427)
#include "stm32f427xx.h"
#else
#error Unrecognised CPU
#endif

#include "displayFile.h"

/* Easy configuration options */
/* ========================== */

#define BUSY_DEBUG                       /* Define this to enable a busy flag */
#define HIGHPRI_IRQ (1)                  /* This is the HSYNC interrupt and needs to be very high priority */
#define LOWPRI_IRQ  (2)                  /* This is the line preparation (SPI) interrupt and can have a */
                                         /* lower priority, but you may have to raise it if you see corruption. */

/* Internals */
/* ========= */

#define XSIZE      58                    /* Number of characters wide the display file is (columns) */
#define YSIZE      18                    /* Number of characters deep the display file is (rows) */
#define YSTRETCH    1                    /* How much extra to stretch in Y per pixel */
#define FRAME_YDISPLACEMENT 10           /* How many visible lines to output before starting to display */


#define ROUNDUP4(x) (((x+3)/4)*4)
#define XEXTENTB   (ROUNDUP4(XSIZE))     /* What the X resolution is in bytes */

#ifdef BUSY_MONITOR
#define BUSYBIT    12
#define BUSYPT     GPIOB
#define SETUP_BUSY do {					\
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		\
    BUSYPT->MODER |= (1<<(2*BUSYBIT));			\
    BUSYPT->OSPEEDR |= (2<<(2*BUSYBIT));	\
  } while (0)
#define AM_IDLE    BUSYPT->BSRR=(1<<BUSYBIT)
#define AM_BUSY    BUSYPT->BSRR=(1<<(BUSYBIT+16))

#else
#define SETUP_BUSY {}
#define AM_IDLE    {}
#define AM_BUSY    {}
#endif

/* ============================================================================================ */

uint32_t vidxSizeG(void);
uint32_t vidySizeG(void);
struct displayFile *vidInit(void);

/* ============================================================================================ */
#endif	/*  _VIDEO_H_ */

