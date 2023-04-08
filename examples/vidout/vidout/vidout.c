/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019-2023 Dave Marples. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * VGA VIDEO Output on a STM32F427VI
 * =================================
 *
 * Pinout;
 *   PA1 = VSYNC (Pin 14 on VGA connector)
 *   PA8 = HSYNC (Pin 13 on VGA connector)
 *   PA7 = Video (Pin 1, 2 or 3 on VGA connector for R, G or B respectively).
 *
 * Note that the spec calls for PA1/PA8 to be 5V and PA7 to be 1V max so you
 * might want to put a series resistor on PA7. The input impedence of VGA is 75R
 * so something in the region of 220R should be fine.
 *
 * To use this subsystem just make sure you've not got anything on the high
 * priority interrupt levels (by default this is configured for the highest two,
 * but the SPI interrupt can have lower priority if you've got other things you
 * need to do).
 *
 * Just call vidInit to start video output and to get an object back that you
 * can manipulate to create output.  Both text and graphic output are supported.
 * See the example main for how to use it, and the API exposed by displayFile
 * too.
 *
 * In general this file should be fire and forget. Once video is up and running
 * it doesn't need any further maintainence or input from you.
 */

#include "vidout.h"
#include "displayFile.h"
#include "rasterLine.h"
#include <stdio.h>

#ifdef MONITOR_OUTPUT
#include "itm_messages.h"
#include "orblcd_protocol.h"
#endif

#include "font-8x16basic.cinc" /* The font to use */

/* Setup materials section */
/* ======================= */

/* Setup pinning and perhiperals to be used ... if these are changed then  */
/* be careful to ensure that clocks/power are enabled to the replacements. */

#define VSYNCBIT 0
#define VSYNCPT GPIOA
#define HSYNCBIT 8
#define HSYNCPT GPIOA
#define VOUTBIT 7
#define VOUTPT GPIOA
#define TIM TIM1
#define TIM_IRQHandler TIM1_CC_IRQHandler
#define SPI SPI1

#define DMA DMA2
#define DMA_CHANNEL DMA2_Stream3
#define DMA_CHANNEL_IRQn DMA2_Stream3_IRQn
#define DMA_CHANNEL_IRQHandler DMA2_Stream3_IRQHandler

#define SETUP_VSYNC                                                            \
  do {                                                                         \
    VSYNCPT->MODER |= (1 << (2 * VSYNCBIT));                                   \
    VSYNCPT->OSPEEDR |= (2 << (2 * BUSYBIT));                                  \
  } while (0) /* Medium Speed, GPIO Output */

#define VSYNC_HIGH VSYNCPT->BSRR |= (1 << VSYNCBIT)
#define VSYNC_LOW VSYNCPT->BSRR |= (1 << (16 + VSYNCBIT))

#define SETUP_HSYNC                                                            \
  do {                                                                         \
    HSYNCPT->MODER |= (2 << (2 * HSYNCBIT));                                   \
    HSYNCPT->OSPEEDR |= (2 << (2 * HSYNCBIT));                                 \
    HSYNCPT->AFR[HSYNCBIT / 8] |= (1 << (4 * (HSYNCBIT % 8)));                 \
  } while (0) /* Medium Speed, Alternate 1 PushPull */

#define SETUP_VOUT                                                             \
  do {                                                                         \
    VOUTPT->MODER |= (2 << (2 * VOUTBIT));                                     \
    VOUTPT->OSPEEDR |= (2 << (2 * VOUTBIT));                                   \
    VOUTPT->AFR[VOUTBIT / 8] |= (5 << (4 * (VOUTBIT % 8)));                    \
  } while (0) /* Medium Speed, Alternate 5 PushPull */

/* Screen definition section */
/* ========================= */

/* What the screen looks like */
#define YEXTENT                                                                \
  (YSIZE * FONTHEIGHT *                                                        \
   (YSTRETCH + 1)) /* How much raster we need to display all of Y */

/* Display protocol material */
/* ========================= */

/* Various elements of the display protocol ... all timing stems from these...
 */
#define FRAME_START 0     /* Start of frame - VSYNC pulse */
#define FRAME_BACKPORCH 2 /* End of VSYNC, start of back porch + blanking */
#define FRAME_BACKPORCH_END 22 /* End of Backportch region */
#define FRAME_OUTPUT_START                                                     \
  (FRAME_BACKPORCH_END + FRAME_YDISPLACEMENT) /* Active frame output start */
#define FRAME_OUTPUT_END                                                       \
  (FRAME_OUTPUT_START + YEXTENT + 1) /* Active frame output end */
#define FRAME_END 624                /* End of frame ... start next one */

/* Clock ticks for each element of a frame (in terms of 168MHz clock pulses) */
#define LINEPERIOD (4799)         /* Length of a line */
#define HORIZSYNCPULSEWIDTH (336) /* Horizontal pulse width */
#define SYNCPLUSPORCH                                                          \
  (850) /* Sync + porch period, adjust if needed to centralise the image */

/* Internal Setup */
/* ============== */

/* Definition of the screen ... done here to avoid it going on the stack */
char storage[DF_SIZE(YSIZE, XSIZE)];

/* Material related to this instance */
/* ================================= */

static volatile struct videoMachine {
  const struct rasterFont *f; /* the font in use */
  struct displayFile *d;      /* The display file being output */
  uint8_t lineBuff[2][ROUNDUP4(XSIZE +
                               2)]; /* Line buffer containing the constructed
                                       raster for output (roundup to word) */
  uint32_t scanLine;    /* The current line being scanned on the screen */
  uint32_t stretchLine; /* Counter for line stretching */
  int32_t opLine;       /* Line of frame being output */
  uint32_t readLine;    /* Line currently being written/read from */
} _v = {.f = &font};

/* TIM_IRQHandler runs _very_ frequently and is time critical. Give it the
 * maximum chance of working well by optimising as much as possible.
 */

/* ============================================================================================
 */

void TIM_IRQHandler(void) {
  /* Called at the end of each scanline to schedule the next element of the
   * protocol Assumes that the next scanline will have been prepared for output
   * already,
   */

  AM_BUSY;

  /* Clear the interrupt and move to the next scanline */
  TIM->SR &= ~TIM_SR_CC2IF;

  /* Remove TCIE and Enable */
  DMA->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
               DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
  DMA_CHANNEL->CR &= ~(DMA_SxCR_EN | DMA_SxCR_TCIE);

  /* Now, depending on what element of the frame we're on, do the magic to
   * output it */
  switch (_v.scanLine++) {
    /* ------------------------------------------------------------------------
     */
  case FRAME_START ... FRAME_BACKPORCH - 1:
    /* Start of frame - create sync pulse */
    VSYNC_HIGH;
    break;

    /* ------------------------------------------------------------------------
     */
  case FRAME_BACKPORCH ... FRAME_OUTPUT_START - 1:
    /* Sync pulse done - top blanking */
    VSYNC_LOW;
    _v.stretchLine = _v.opLine = _v.readLine = 0;

    /* Send out a zeroed line... this is in the first lineBuff at the moment */
    DMA_CHANNEL->M0AR = (uint32_t)_v.lineBuff[1];
    DMA_CHANNEL->NDTR = XSIZE;
    DMA_CHANNEL->CR |= DMA_SxCR_EN; /* Enable, No TCIE */
    break;

    /* ------------------------------------------------------------------------
     */
  case FRAME_OUTPUT_START ... FRAME_OUTPUT_END:
    /* Set the previusly prepared scanLine ready to be output */
    DMA_CHANNEL->M0AR = (uint32_t)_v.lineBuff[_v.readLine];
    /* Send two extra chars for end of line blanking */
    DMA_CHANNEL->NDTR = XSIZE + 2;

    /* See if it's time for the next line to be generated */
    if (_v.stretchLine++ == YSTRETCH) {
      /* Next time we'll swap sides, so we can write into the current buffer */
      /* once it's been transmitted */
      DMA_CHANNEL->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE; /* TCIE and Enable */

      /* Read from the (hopefully) already prepared line, and reset the stretch
       */
      _v.readLine = !_v.readLine;
      _v.stretchLine = 0;
    } else {
      DMA_CHANNEL->CR |= DMA_SxCR_EN; /*  Enable */
    }
    break;

    /* ------------------------------------------------------------------------
     */
  case FRAME_OUTPUT_END + 1 ... FRAME_END - 1:
    /* Send out a zeroed line... */
    DMA_CHANNEL->M0AR = (uint32_t)_v.lineBuff[1];
    DMA_CHANNEL->NDTR = XSIZE;
    DMA_CHANNEL->CR |= DMA_SxCR_EN;
    break;

    /* ------------------------------------------------------------------------
     */
  case FRAME_END:
    /* End of frame */
    _v.scanLine = 0;
    break;
  }

  AM_IDLE;
}
/* ============================================================================================
 */

void DMA_CHANNEL_IRQHandler(void)

{
  /* This routine is called immediately there is room to calculate the next line
   * for output */
  /* It is called as a low priority interrupt so it can do it's work when
   * there's time.     */
  AM_BUSY;

  DMA->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
               DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

  if (_v.opLine >= YSIZE * FONTHEIGHT) {
    /* No more valid scan lines in this frame, so don't output more video */
    /* ...and make sure the first line is set up to go out */
    rasterLine(_v.d, _v.f, (uint32_t *)_v.lineBuff[0], 0);
    _v.readLine = 0;

#ifdef MONITOR_OUTPUT
    if (_v.opLine == YSIZE * FONTHEIGHT) {
      /* This is sent at the start of every frame in case the other end wasn't
       * awake */
      ITM_Send32(LCD_COMMAND_CHANNEL,
                 ORBLCD_OPEN_SCREEN(XSIZE * 8, YSIZE * 16, ORBLCD_DEPTH_1));
    }
#endif

    /* Zero out line 1 as this will be used for blanking */
    for (uint32_t t = 0; t < XSIZE; t++)
      _v.lineBuff[1][t] = 0;
  } else {
    /* Prepare next line for output */
    rasterLine(_v.d, _v.f, (uint32_t *)_v.lineBuff[!_v.readLine], _v.opLine++);
  }
  AM_IDLE;
}

/* ============================================================================================
 */
/* ============================================================================================
 */
/* ============================================================================================
 */
/* Public routines */
/* ============================================================================================
 */
/* ============================================================================================
 */
/* ============================================================================================
 */

uint32_t vidxSizeG(void) { return YSIZE * FONTHEIGHT; }

/* ============================================================================================
 */

uint32_t vidySizeG(void) { return XSIZE * FONTWIDTH; }

/* ============================================================================================
 */

struct displayFile *vidInit(void)

{
  /* Switch on power/clocks to required perhiperals */
  // RCC->AHBENR |= RCC_AHBPeriph_DMA1;
  RCC->AHB1ENR |=
      RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN;
  // RCC->APB2ENR |= RCC_APB2Periph_SPI1 | RCC_APB2Periph_TIM1 |
  // RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB;
  RCC->APB2ENR |=
      RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_SYSCFGEN;

  SETUP_BUSY;
  SETUP_VSYNC;
  SETUP_HSYNC;
  SETUP_VOUT;

  /* Create the video handler object */
  _v.d = DF_create(YSIZE, XSIZE, storage, ' ');

  /* Setup the DMA transfer details */
  DMA_CHANNEL->PAR = (uint32_t)&SPI->DR;
  DMA_CHANNEL->CR = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_DIR_0 |
                    DMA_SxCR_MINC | DMA_SxCR_TCIE;

  /* Setup the SPI transfer details */
  SPI->CR2 |= SPI_CR2_TXDMAEN;
  SPI->CR1 =
      SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;

  /* Set up timings for line period and horizontal pulses */
  TIM->ARR = LINEPERIOD;
  TIM->CCR1 = HORIZSYNCPULSEWIDTH; /* Set pulse to end */
  TIM->CCR2 = SYNCPLUSPORCH; /* Set secondary pulse, which generates interrupt,
                                further into line */

  TIM->CCMR1 = TIM_CCMR1_OC1M_1 |
               TIM_CCMR1_OC1M_2; /* PWM mode 1 (Ch1 active until triggered) */
  TIM->CCER = TIM_CCER_CC1E;     /* CH1 Output Enable, active High */
  TIM->BDTR = TIM_BDTR_MOE;      /* Master output enable */
  TIM->SMCR = TIM_SMCR_MSM;      /* Delay trigger for perfect sync */
  TIM->DIER = TIM_DIER_CC2IE;    /* Interrupt on channel 2 match only */

  /* Now setup the interrupts... */
  NVIC_SetPriorityGrouping(0U);

  /* Interrupt TIM1 */
  NVIC_SetPriority(TIM1_CC_IRQn, HIGHPRI_IRQ);
  NVIC_EnableIRQ(TIM1_CC_IRQn);

  /* ...and prepare the DMA interrupt for refreshing the line buffer */
  NVIC_SetPriority(DMA_CHANNEL_IRQn, LOWPRI_IRQ);
  NVIC_EnableIRQ(DMA_CHANNEL_IRQn);

  TIM->CR1 = TIM_CR1_CEN; /* timer1 (Line timer) run */

  return _v.d;
}

/* ============================================================================================
 */
