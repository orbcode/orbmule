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

/* Pinning */
/* ======= */
/* Setup pinning and perhiperals to be used ... if these are changed then  */
/* be careful to ensure that clocks/power are enabled to the replacements. */

#define VSYNCBIT 0
#define VSYNCPT GPIOA
#define VSYNCBITP 1

#define HSYNCBIT 8
#define HSYNCPT GPIOA

#define VOUTBIT 7
#define VOUTPT GPIOA
#define VOUTBITP 7

#define BUSYBIT    14
#define BUSYPT     GPIOD
#define BUSYBITP   6

#define TIM TIM1
#define TIM_IRQHandler TIM1_CC_IRQHandler
#define SPI SPI1

#define DMA DMA2
#define DMA_CHANNEL DMA2_Stream3
#define DMA_CHANNEL_IRQn DMA2_Stream3_IRQn
#define DMA_CHANNEL_IRQHandler DMA2_Stream3_IRQHandler

#define SETUP_VSYNC                                                            \
  do {                                                                         \
    VSYNCPT->MODER   |= (1 << (2 * VSYNCBIT));                                 \
    VSYNCPT->OSPEEDR |= (2 << (2 * VSYNCBIT));                                 \
    VSYNCPT->MODER   |= (1 << (2 * VSYNCBITP));                                \
    VSYNCPT->OSPEEDR |= (2 << (2 * VSYNCBITP));                                \
  } while (0) /* Medium Speed, GPIO Output */

#define VSYNC_HIGH VSYNCPT->BSRR |= (1 << VSYNCBIT) | (1 << VSYNCBITP)
#define VSYNC_LOW VSYNCPT->BSRR |= (1 << (16 + VSYNCBIT)) | (1 << (16 + VSYNCBITP))

#define SETUP_HSYNC                                                            \
  do {                                                                         \
    HSYNCPT->MODER |= (2 << (2 * HSYNCBIT));                                   \
    HSYNCPT->OSPEEDR |= (2 << (2 * HSYNCBIT));                                 \
    HSYNCPT->AFR[HSYNCBIT / 8] |= (1 << (4 * (HSYNCBIT % 8)));                 \
  } while (0) /* Medium Speed, Alternate 1 PushPull */

#define SETUP_VOUT                                                             \
  do {                                                                         \
    VOUTPT->MODER            |= (2 << (2 * VOUTBIT));                          \
    VOUTPT->OSPEEDR          |= (1 << (2 * VOUTBIT));                          \
    VOUTPT->AFR[VOUTBIT / 8] |= (5 << (4 * (VOUTBIT % 8)));                    \
    VOUTPT->MODER            |= (2 << (2 * VOUTBITP));                         \
    VOUTPT->OSPEEDR          |= (1 << (2 * VOUTBITP));                         \
    VOUTPT->AFR[VOUTBIT / 8] |= (5 << (4 * (VOUTBITP % 8)));                   \
  } while (0) /* Medium Speed, Alternate 5 PushPull */

#ifdef BUSY_MONITOR
#define SETUP_BUSY do {					            \
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;		\
    BUSYPT->MODER |= (1<<(2*BUSYBIT));			\
    BUSYPT->OSPEEDR |= (2<<(2*BUSYBIT));	  \
    BUSYPT->MODER |= (1<<(2*BUSYBITP));			\
    BUSYPT->OSPEEDR |= (2<<(2*BUSYBITP));	  \
  } while (0)
#define AM_BUSY    BUSYPT->BSRR=(1<<BUSYBIT) | (1<<BUSYBITP)
#define AM_IDLE    BUSYPT->BSRR=(1<<(BUSYBIT+16)) | (1<<(BUSYBITP+16))
#else
#define SETUP_BUSY {}
#define AM_IDLE    {}
#define AM_BUSY    {}
#endif

/* Easy configuration options */
/* ========================== */

#define BUSY_DEBUG                       /* Define this to enable a busy flag */
#define HIGHPRI_IRQ (1)                  /* This is the HSYNC interrupt and needs to be very high priority */
#define LOWPRI_IRQ  (2)                  /* This is the line preparation (SPI) interrupt and can have a */
                                         /* lower priority, but you may have to raise it if you see corruption. */

#define LOCATION_OPTIMISE 
//__attribute__((__section__(".ramprog")))

/* Internals */
/* ========= */

#define XSIZE      58                    /* Number of characters wide the display file is (columns) */
#define YSIZE      18                    /* Number of characters deep the display file is (rows) */
#define YSTRETCH    1                    /* How much extra to stretch in Y per pixel */
#define FRAME_YDISPLACEMENT 10           /* How many visible lines to output before starting to display */


#define ROUNDUP4(x) (((x+3)/4)*4)
#define XEXTENTB   (ROUNDUP4(XSIZE))     /* What the X resolution is in bytes */

/* ============================================================================================ */

uint32_t vidxSizeG(void);
uint32_t vidySizeG(void);
struct displayFile *vidInit(void);

/* ============================================================================================ */
#endif	/*  _VIDEO_H_ */

