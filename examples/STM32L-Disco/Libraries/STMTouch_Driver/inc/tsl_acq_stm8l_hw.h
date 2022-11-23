/**
  ******************************************************************************
  * @file    tsl_acq_stm8l_hw.h
  * @author  MCD Application Team
  * @version V1.4.4
  * @date    31-March-2014
  * @brief   This file contains external declarations of the tsl_acq_stm8l_hw.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TSL_ACQ_STM8L_HW_H
#define __TSL_ACQ_STM8L_HW_H

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "tsl_conf_stm8l.h"
#include "tsl_types.h"

/* Defines -------------------------------------------------------------------*/

#ifndef CONST
    #define CONST const
#endif

enum
{
    GR1 = 0,
    GR2,
    GR3,
    GR4,
    GR5,
    GR6,
    GR7,
    GR8
};

enum
{
    TSL_BANK_GPIOA = 0,
    TSL_BANK_GPIOB,
    TSL_BANK_GPIOC,
    TSL_BANK_GPIOD,
    TSL_BANK_GPIOE
};

/** GPIOs list:
  - bits 7:3 GPIO number (0=GPIOA, 1=GPIOB, 2=GPIOC, 3=GPIOD, 4=GPIOE)
  - bits 2:0 IO number (0=pin0, ..., 7=pin7)
  */

// GROUP1
#define  PA4 (0x00|0x04) // CH1
#define  PA5 (0x00|0x05) // CH2
#define  PA6 (0x00|0x06) // CH3
#define  PA7 (0x00|0x07) // CH4
// GROUP2
#define  PC7 (0x10|0x07) // CH1
#define  PC4 (0x10|0x04) // CH2
#define  PC3 (0x10|0x03) // CH3
#define  PE7 (0x20|0x07) // CH4
// GROUP3
#define  PC2 (0x10|0x02) // CH1
#define  PD7 (0x18|0x07) // CH2
#define  PD6 (0x18|0x06) // CH3
// GROUP4
#define  PD5 (0x18|0x05) // CH1
#define  PD4 (0x18|0x04) // CH2
#define  PB7 (0x08|0x07) // CH3
// GROUP5
#define  PB6 (0x08|0x06) // CH1
#define  PB5 (0x08|0x05) // CH2
#define  PB4 (0x08|0x04) // CH3
// GROUP6
#define  PB3 (0x08|0x03) // CH1
#define  PB2 (0x08|0x02) // CH2
#define  PB1 (0x08|0x01) // CH3
// GROUP7
#define  PB0 (0x08|0x00) // CH1
#define  PD3 (0x18|0x03) // CH2
#define  PD2 (0x18|0x02) // CH3
#define  PE3 (0x20|0x03) // CH4
// GROUP8
#define  PD1 (0x18|0x01) // CH1
#define  PD0 (0x18|0x00) // CH2
#define  PE5 (0x20|0x05) // CH3
#define  PE4 (0x20|0x04) // CH4

// Acquisition pulses period
/** Master timer reload value for HW acquisition only (range=4..65534, even number)
    --> Period for Charge/Transfer cycle = ((TSLPRM_TIM_RELOAD*2)/FTimer)
*/
#define TIM_RELOAD ((TSLPRM_CT_PERIOD * TSLPRM_TIMER_FREQ) / 2)
#define TIM2_PWM_CH1_WIDTH  ((TIM_RELOAD / 2) - 1) // Configure channel 1 Pulse Width
#define TIM2_PWM_CH2_WIDTH  ((TIM_RELOAD / 2) + 1) // Configure channel 2 Pulse Width

/* Exported types ------------------------------------------------------------*/

// For all devices/acquisitions

typedef uint16_t  TSL_tMeas_T; /**< Measurement */
typedef uint16_t  TSL_tRef_T; /**< Reference */
typedef int16_t   TSL_tDelta_T; /**< Delta */

typedef uint8_t   TSL_tIndexSrc_T; /**< Channel source index */
typedef uint8_t   TSL_tIndexDest_T; /**< Channel destination index */

typedef uint8_t   TSL_tRefRest_T; /**< Reference Rest (ECS) */
typedef uint16_t  TSL_tKCoeff_T; /**< K coefficient (ECS) */

typedef uint16_t  TSL_tIndex_T; /**< Generic index */
typedef uint16_t  TSL_tNb_T; /**< Generic number */
typedef uint8_t   TSL_tCounter_T; /**< Generic counter used for debounce */

typedef uint8_t   TSL_tThreshold_T; /**< Delta threshold */

typedef int16_t   TSL_tsignPosition_T; /**< Linear and Rotary sensors position */
typedef uint8_t   TSL_tPosition_T; /**< Linear and Rotary sensors position */

typedef uint16_t  TSL_tTick_ms_T; /**< Time in ms */
typedef uint8_t   TSL_tTick_sec_T; /**< Time in sec */

//------------------------------------------------------------------------------
// Channel
//------------------------------------------------------------------------------

typedef uint8_t TSL_Conf_T;

/** Channel destination index
  */
typedef struct
{
    TSL_tIndexDest_T  IdxDest; /**< Index in the Channel data array */
} TSL_ChannelDest_T;

/** Channel Source and Configuration
  */
typedef struct
{
    TSL_tIndex_T IdxSrc; /**< Index of source value */
    // For stm8l acquisition only
    TSL_Conf_T   sampling; /**< Indicates which GPIO.n is used for the sample */
    TSL_Conf_T   channel;   /**< Indicates which GPIO.n is used for the channel */
} TSL_ChannelSrc_T;

/** Channel flags
  */
typedef struct
{
    unsigned int DataReady : 1; /**< To identify a new measurement (TSL_DataReady_enum_T) */
    unsigned int AcqStatus : 2; /**< Acquisition status (TSL_AcqStatus_enum_T) */
    unsigned int ObjStatus : 2; /**< Object status (TSL_ObjStatus_enum_T) */
} TSL_ChannelFlags_T;

/** Channel Data
  */
typedef struct
{
    TSL_ChannelFlags_T   Flags;   /**< Flags */
    TSL_tRef_T           Ref;     /**< Reference */
    TSL_tRefRest_T       RefRest; /**< Reference rest for ECS */
    TSL_tDelta_T         Delta;   /**< Delta */
#if TSLPRM_USE_MEAS > 0
    TSL_tMeas_T          Meas;    /**< Hold the last acquisition measure */
#endif
} TSL_ChannelData_T;

//------------------------------------------------------------------------------
// Bank
//------------------------------------------------------------------------------

/** Bank
  */
typedef struct
{
    // Common to all acquisitions
    CONST TSL_ChannelSrc_T  *p_chSrc;     /**< Pointer to the Channel Source and Configuration */
    CONST TSL_ChannelDest_T *p_chDest;    /**< Pointer to the Channel Destination */
    TSL_ChannelData_T       *p_chData;    /**< Pointer to the Channel Data */
    TSL_tNb_T               NbChannels;   /**< Number of channels in the bank */
    // For stm8l acquisition only
    TSL_Conf_T              shield_sampling; /**< Indicates which GPIO.n is used for the shield sample */
    TSL_Conf_T              shield_channel;   /**< Indicates which GPIO.n is used for the shield channel */
} TSL_Bank_T;

/** Bank Config Mask
 */
typedef struct
{
    uint8_t  ch1;  /**< Contains the mask for all first channel of group */
    uint8_t  ch2;  /**< Contains the mask for all second channel of group */
    uint8_t  ch3;  /**< Contains the mask for all third channel of group */
    uint8_t  ch4;  /**< Contains the mask for all fourth channel of group (if LD device)*/
} TSL_Bank_Config_Mask_T;

/* Exported variables --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
TSL_Status_enum_T TSL_acq_Init(void);
TSL_Status_enum_T TSL_acq_BankConfig(TSL_tIndex_T idx_bk);
void TSL_acq_BankStartAcq(void);
TSL_Status_enum_T TSL_acq_BankWaitEOC(void);
void TSL_CT_HWacq_TIM3(void);
void TSL_CT_HWacq_RI(void);
TSL_tMeas_T TSL_acq_GetMeas(TSL_tIndex_T index);
TSL_AcqStatus_enum_T TSL_acq_CheckNoise(void);

TSL_Bool_enum_T TSL_acq_UseFilter(TSL_ChannelData_T *pCh);
TSL_tDelta_T TSL_acq_ComputeDelta(TSL_tRef_T ref, TSL_tMeas_T meas);
TSL_tMeas_T TSL_acq_ComputeMeas(TSL_tRef_T ref, TSL_tDelta_T delta);
TSL_Bool_enum_T TSL_acq_TestReferenceOutOfRange(TSL_ChannelData_T *pCh);
TSL_Bool_enum_T TSL_acq_TestFirstReferenceIsValid(TSL_ChannelData_T *pCh, TSL_tMeas_T new_meas);

#endif /* __TSL_ACQ_STM8L_HW_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
