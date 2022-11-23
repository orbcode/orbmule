/**
  ******************************************************************************
  * @file    tsl_time.h
  * @author  MCD Application Team
  * @version V1.4.4
  * @date    31-March-2014
  * @brief   This file contains external declarations of the tsl_time.c file.
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
#ifndef __TSL_TIME_H
#define __TSL_TIME_H

/* Includes ------------------------------------------------------------------*/

#if defined(STM8L10X) || defined(STM8L15X_MD) || defined(STM8L15X_MDP) || defined(STM8L15X_HD)
    #include "tsl_acq_stm8l_sw.h" // Software acquisition
    #include "tsl_time_stm8l.h"
#endif

#if defined(STM8L15X_LD)
    #if defined(TSLPRM_STM8L1XX_SW_ACQ)
        #include "tsl_acq_stm8l_sw.h" // Software acquisition
    #else
        #include "tsl_acq_stm8l_hw.h" // Hardware acquisition with Timers (default)
    #endif
    #include "tsl_time_stm8l.h"
#endif

#if defined(STM8TL5X)
    #include "tsl_acq_stm8tl5x.h"
    #include "tsl_time_stm8tl5x.h"
#endif

#if defined(STM32L1XX_MD)
    #include "tsl_acq_stm32l1xx_sw.h" // Software acquisition only
    #include "tsl_time_stm32l1xx.h"
#endif

#if defined(STM32L1XX_MDP) || defined(STM32L1XX_HD) || defined(STM32L1XX_XL)
    #if defined(TSLPRM_STM32L1XX_SW_ACQ)
        #include "tsl_acq_stm32l1xx_sw.h" // Software acquisition
    #else
        #include "tsl_acq_stm32l1xx_hw.h" // Hardware acquisition with Timers (default)
    #endif
    #include "tsl_time_stm32l1xx.h"
#endif

#if defined(STM32F0XX) || defined(STM32F0XX_MD) || defined(STM32F0XX_HD) ||\
    defined(STM32F051) || defined(STM32F072) || defined(STM32F042)
    #include "tsl_acq_stm32f0xx.h"
    #include "tsl_time_stm32f0xx.h"
#endif

#if defined(STM32F303xC) || defined(STM32F334x8) || defined(STM32F303x8) || defined(STM32F301x8) || defined(STM32F302x8) ||\
    defined(STM32F37X)
    #include "tsl_acq_stm32f3xx.h"
    #include "tsl_time_stm32f3xx.h"
#endif

/* Exported functions ------------------------------------------------------- */

void TSL_tim_ProcessIT(void);
TSL_Status_enum_T TSL_tim_CheckDelay_ms(TSL_tTick_ms_T delay_ms, __IO TSL_tTick_ms_T *last_tick);
TSL_Status_enum_T TSL_tim_CheckDelay_sec(TSL_tTick_sec_T delay_sec, __IO TSL_tTick_sec_T *last_tick);
void TSL_CallBack_TimerTick(void);

#endif /* __TSL_TIME_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
