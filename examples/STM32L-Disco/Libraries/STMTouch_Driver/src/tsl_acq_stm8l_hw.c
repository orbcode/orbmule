/**
  ******************************************************************************
  * @file    tsl_acq_stm8l_hw.c
  * @author  MCD Application Team
  * @version V1.4.4
  * @date    31-March-2014
  * @brief   This file contains all functions to manage the acquisition
  *          on STM8L products using the hardware acquisition mode (with Timers).
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

/* Includes ------------------------------------------------------------------*/
#include "tsl_acq_stm8l_hw.h"
#include "tsl_globals.h"

/* Private typedefs ----------------------------------------------------------*/

/** Structure containing RI IO informations according to GPIO.
  */
typedef struct
{
    unsigned int IO_Channel : 4; /**< Channel number from 1 to 4 in the Routing interface group */
    unsigned int IO_Group   : 4; /**< Group number in the Routing interface */
} TSL_IOConf_T;

/* Private defines -----------------------------------------------------------*/

#define MAX_CHANNEL_NUMBER_BY_GROUP (4)

/* Private macros ------------------------------------------------------------*/

#define GPIO_PORT(GPIO)  (GPIO >> 3)   /**< Get the GPIO port*/
#define GPIO_BIT(GPIO)   (GPIO & 0x07) /**< Get the GPIO pin number*/

#define IS_BANK_INDEX_OK(INDEX)   (((INDEX) == 0) || (((INDEX) > 0) && ((INDEX) < TSLPRM_TOTAL_BANKS))) /**< Check if the index have a good range*/

#define GPIO_ODR_HIGH(GPIO)      (p_GPIOx[GPIO_PORT(GPIO)]->ODR |= (uint8_t)(1 << GPIO_BIT(GPIO)))
#define GPIO_ODR_LOW(GPIO)       (p_GPIOx[GPIO_PORT(GPIO)]->ODR &= (uint8_t)(~(1 << GPIO_BIT(GPIO))))
#define GPIO_DDR_IN(GPIO)        (p_GPIOx[GPIO_PORT(GPIO)]->DDR &= (uint8_t)(~(1 << GPIO_BIT(GPIO))))
#define GPIO_DDR_OUT(GPIO)       (p_GPIOx[GPIO_PORT(GPIO)]->DDR |= (uint8_t)(1 << GPIO_BIT(GPIO)))
#define GPIO_CR1_PP(GPIO)        (p_GPIOx[GPIO_PORT(GPIO)]->CR1 |= (uint8_t)(1 << GPIO_BIT(GPIO)))
#define GPIO_CR1_FLOATING(GPIO)  (p_GPIOx[GPIO_PORT(GPIO)]->CR1 &= (uint8_t)(~(1 << GPIO_BIT(GPIO))))

#define DISABLE_MASK(GPIO)     (DisableMask[(GPIO_to_SW_Conf[GPIO].IO_Channel)-1] |= (uint8_t)(1 << GPIO_to_SW_Conf[GPIO].IO_Group)) /**< Create disable mask array to modify initial bank mask before acquisition (only for STATUS_OFF)*/
#define DISABLE_SAMPLING(GPIO) (DisableSampling |= (uint8_t)(1 << GPIO_to_SW_Conf[GPIO].IO_Group)) /**< Create disable sampling mask to don't take sampling measurement of corresponding channels(for STATUS_BURST_ONLY and shield) */

/* Private variables ---------------------------------------------------------*/

__IO uint8_t *p_IOIRx;            // Pointer to the IOIRx register (x from 1 to 4)
__IO uint8_t *p_IOMRx;            // Pointer to the IOMRx register (x from 1 to 4)
uint8_t OldStatus;                // Mask used to memorize the IOIRx bits processed during the acquisition
uint8_t BankDone;                // Control if all activate sampling reach the VIH level
uint8_t CurrentSampling;         // Mask to control IOGCR register
uint8_t CurrentChannel;          // Mask to control IOGCR register
uint8_t ChannelSampling;         // Contain the channel number where all sampling are connected
uint8_t DisableSampling;         // Disable sampling mask when the Burst Only mode is activated for one channel of the current bank(not get the measure)

TSL_Bank_Config_Mask_T
BankMask[TSLPRM_TOTAL_BANKS]; // Complete masks (channel and sampling) to configure IOCMRx and IOSRx registers for all banks
uint8_t SamplingMask[TSLPRM_TOTAL_BANKS];            // Sampling mask to configure IOGCR register for all banks
uint8_t ChannelMask[TSLPRM_TOTAL_BANKS];             // Channel mask to configure IOGCR register for all banks
uint8_t DisableMask[MAX_CHANNEL_NUMBER_BY_GROUP];    // Complete disable mask(channel and sampling) when the Channel OFF mode is activated for one channel of the current bank(to modifie the Current_Bank)
uint8_t CurrentBank[MAX_CHANNEL_NUMBER_BY_GROUP];    // Complete mask for the current bank
uint16_t tab_MeasurementCounter[8] = {0};             // Measurement of each sampling of the current bank

TSL_Status_enum_T TSL_Acq_Status = TSL_STATUS_BUSY;

GPIO_TypeDef *p_GPIOx[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF};

__IO uint8_t *RI_IOIRx_Register[MAX_CHANNEL_NUMBER_BY_GROUP] = {&(RI->IOIR1), &(RI->IOIR2), &(RI->IOIR3), &(RI->IOIR4)};
__IO uint8_t *RI_IOMRx_Register[MAX_CHANNEL_NUMBER_BY_GROUP] = {&(RI->IOMR1), &(RI->IOMR2), &(RI->IOMR3), &(RI->IOMR4)};


/* Table which do the link between GPIO and switch configuation:{x,y}
   x = channel number
   y = group number - 1
   Note: {0,0} = not connect to IO switch
*/
CONST TSL_IOConf_T GPIO_to_SW_Conf[40] =
{
    // Port A definitions
    {0, 0}, // PA0
    {0, 0}, // PA1
    {0, 0}, // PA2
    {0, 0}, // PA3
    {3, 0}, // PA4 is channel 3 of Group 1
    {2, 0}, // PA5 is channel 2 of Group 1
    {1, 0}, // PA6 is channel 1 of Group 1
    {4, 0}, // PA7 is channel 4 of Group 1
    // Port B definitions
    {1, 6}, // PB0
    {3, 5},
    {2, 5},
    {1, 5},
    {3, 4},
    {2, 4},
    {1, 4},
    {3, 3}, // PB7
    // Port C definitions
    {0, 0}, // PC0
    {0, 0},
    {1, 2},
    {3, 1},
    {2, 1},
    {0, 0},
    {0, 0},
    {1, 1}, // PC7
    // Port D definitions
    {2, 7}, // PD0
    {1, 7},
    {3, 6},
    {2, 6},
    {2, 3},
    {1, 3},
    {3, 2},
    {2, 2}, // PD7
    // Port E definitions
    {0, 0}, // PE0
    {0, 0},
    {0, 0},
    {4, 6},
    {4, 7},
    {3, 7},
    {0, 0},
    {4, 1} // PE7
};

/* Private functions prototype -----------------------------------------------*/
void SoftDelay(uint16_t val);
void CreateMask(uint16_t idx_bk, uint8_t GPIO);
void TSL_Init_GPIOs(void);
void TSL_Init_TIMs(void);
TSL_Status_enum_T TSL_Init_RI(void);


/**
  * @brief  Initializes the touch sensing GPIOs.
  * @param  None
  * @retval None
  */
void TSL_Init_GPIOs(void)
{
    CONST TSL_Bank_T *p_bank = &(TSL_Globals.Bank_Array[0]); // Pointer to the first bank
    CONST TSL_ChannelSrc_T *p_chSrc = p_bank->p_chSrc; // Pointer to the source channel of the current bank
    uint16_t number_of_channels = 0;
    uint16_t idx_bk;
    uint16_t idx_ch;

    // Initializes each bank and configures the used GPIO
    for (idx_bk = 0; idx_bk < TSLPRM_TOTAL_BANKS; idx_bk++)
        {

            p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
            p_chSrc = p_bank->p_chSrc;

            number_of_channels = p_bank->NbChannels;

#if (TSLPRM_USE_SHIELD > 0)
            // GPIO in Output
            GPIO_DDR_OUT(p_bank->shield_sampling);
            GPIO_DDR_OUT(p_bank->shield_channel);
            // GPIO in PP
            GPIO_CR1_PP(p_bank->shield_sampling);
            GPIO_CR1_PP(p_bank->shield_channel);
            // Output in Low level
            GPIO_ODR_LOW(p_bank->shield_sampling);
            GPIO_ODR_LOW(p_bank->shield_channel);
#endif

            // Initialize the mask for channel and sampling
            for (idx_ch = 0; idx_ch < number_of_channels; idx_ch++)
                {
                    // GPIO are configured in PP Low mode when inactive
                    // GPIO in Output
                    GPIO_DDR_OUT(p_chSrc->sampling);
                    GPIO_DDR_OUT(p_chSrc->channel);
                    // GPIO in PP
                    GPIO_CR1_PP(p_chSrc->sampling);
                    GPIO_CR1_PP(p_chSrc->channel);
                    // Output in Low level
                    GPIO_ODR_LOW(p_chSrc->sampling);
                    GPIO_ODR_LOW(p_chSrc->channel);
                    // Next channel
                    p_chSrc++;
                }
        }
}


/**
  * @brief  Initializes the timers used for touch sensing hardware acquisition.
  * @param  None
  * @retval None
  */
void TSL_Init_TIMs(void)
{
    CLK->PCKENR1 |= 0x03; // Enable TIM2 and TIM3 clocks

    //==============================
    // TIMER 2 configuration: Master
    //==============================
    // Channel 1 as output, set PWM mode 1
    TIM2->CCMR1 = 0x60;
    TIM2->CCMR2 = 0x60;
    // Main Output Enable
    TIM2->BKR |= 0x80; // MOE=1
    // Center-Aligned mode 3
    TIM2->CR1 |= 0x60; // CMS=11
    // OC2 polarity = active low
    TIM2->CCER1 |= 0x20; // CC2P=1
    // Enable OC2
    TIM2->CCER1 |= 0x10; // CC2E=1
    // Set the Prescaler value
    TIM2->PSCR = 0; // fCK_CNT = 16MHz/(0+1) = 16MHz --> T=62.5ns
    // Set the Autoreload value (signal frequency)
    TIM2->ARRH = (uint8_t)(TIM_RELOAD >> 8);
    TIM2->ARRL = (uint8_t)(TIM_RELOAD);
    // Set PWM1 duty cycle
    TIM2->CCR1H = (uint8_t)(TIM2_PWM_CH1_WIDTH >> 8);
    TIM2->CCR1L = (uint8_t)(TIM2_PWM_CH1_WIDTH);
    // Set PWM2 duty cycle
    TIM2->CCR2H = (uint8_t)(TIM2_PWM_CH2_WIDTH >> 8);
    TIM2->CCR2L = (uint8_t)(TIM2_PWM_CH2_WIDTH);
    // Select Master mode, Internal Trigger selection, Gated mode
    TIM2->SMCR = 0x35; // TS=011=ITR3(TIM2), SMS=101=Gated mode enabled
    // Map OC1REF to TRGO
    TIM2->CR2 = 0x40; // MMS=100
    // Enable OC1
    TIM2->CCER1 |= 0x01; // CC1E=1
    // Set Update generation
    TIM2->EGR |= 0x01; // UG=1
    // Set Break interrupt flag
    TIM2->SR1 |= 0x80;

    //==============================
    // TIMER 3 configuration: slave
    //==============================
    // Enable External Clock mode 2, external trigger filter, trigger on high level or rising edge
    TIM3->ETR = 0x42; // ETP=0, ECE=1, ETF=0010
    // Capture/Compare 1 configured as Input: h/w detection mapped on TI2FP1
    TIM3->CCMR1 = 0x02; // CC1S=10
    // Capture/Compare 2 configured as Output: MaxCount
    TIM3->CCMR2 = 0; // CC2S=00
    // Enable CC1 channel as Input for Capture function
    TIM3->CCER1 = 0x01; // CC1E=1
    // Enable counter (slave must be enabled first)
    TIM3->CR1 |= 0x01; // CEN=1
}


/**
  * @brief  Init routing interface.
  * @param  None
  * @retval None
  */
TSL_Status_enum_T TSL_Init_RI(void)
{
    CONST TSL_Bank_T *p_bank = &(TSL_Globals.Bank_Array[0]); // Pointer to the first bank
    CONST TSL_ChannelSrc_T *p_chSrc = p_bank->p_chSrc; // Pointer to the source channel of the current bank
    uint16_t number_of_channels = 0;
    uint16_t idx_bk;
    uint16_t idx_ch;

    // Enable comparator clock to activate the RI block
    CLK->PCKENR2 |= CLK_PCKENR2_COMP;

    // Enable H/W acquisition sequence
    RI->CR |= 0x04; // AM=1

    // Enable Channel Acquisition interrupt
    RI->CR |= 0x01; // TIE=1

    // Suspend Timer2 on h/w detection
    RI->CR |= 0x08; // THALT=1

    // Enable schmitt trigger required for H/W acq mode.
    COMP->CSR1 |= 0x04; // STE=1

    // Initializes each bank and configures the used GPIO
    for (idx_bk = 0; idx_bk < TSLPRM_TOTAL_BANKS; idx_bk++)
        {

            p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
            p_chSrc = p_bank->p_chSrc;

            number_of_channels = p_bank->NbChannels;

            // Masks initialisation
            BankMask[idx_bk].ch1 = 0;
            BankMask[idx_bk].ch2 = 0;
            BankMask[idx_bk].ch3 = 0;
            BankMask[idx_bk].ch4 = 0;

            // Get which channel is used for sampling only one time because it's the same for each couple
            SamplingMask[idx_bk] = (uint8_t)GPIO_to_SW_Conf[p_chSrc->sampling].IO_Channel;

#if (TSLPRM_USE_SHIELD > 0)
            // Create Mask per bank
            CreateMask(idx_bk,p_bank->shield_sampling);
            CreateMask(idx_bk,p_bank->shield_channel);
            ChannelMask[idx_bk] |= (uint8_t)(3 << (2 * ((GPIO_to_SW_Conf[p_bank->shield_channel].IO_Channel) - 1)));
            if ((SamplingMask[idx_bk] != (uint8_t)GPIO_to_SW_Conf[p_bank->shield_sampling].IO_Channel))
                {
                    return TSL_STATUS_ERROR;
                }
#endif

            // Initializes the mask for channel and sampling
            for (idx_ch = 0; idx_ch < number_of_channels; idx_ch++)
                {
                    // Create Mask per bank
                    CreateMask(idx_bk,p_chSrc->channel);
                    CreateMask(idx_bk,p_chSrc->sampling);
                    ChannelMask[idx_bk] |= (uint8_t)(3 << (2 * ((GPIO_to_SW_Conf[p_chSrc->channel].IO_Channel) - 1)));
                    if ((SamplingMask[idx_bk] != (uint8_t)GPIO_to_SW_Conf[p_chSrc->sampling].IO_Channel))
                        {
                            return TSL_STATUS_ERROR;
                        }
                    // Next channel
                    p_chSrc++;
                }

            // Unlock IO to RI register : IO controlled by GPIO
            RI->IOCMR1 &= (uint8_t)(~BankMask[idx_bk].ch1);
            RI->IOCMR2 &= (uint8_t)(~BankMask[idx_bk].ch2);
            RI->IOCMR3 &= (uint8_t)(~BankMask[idx_bk].ch3);
            RI->IOCMR4 &= (uint8_t)(~BankMask[idx_bk].ch4);
        }
    return  TSL_STATUS_OK;
}


/**
  * @brief  Create Mask for all banks
  * @param[in] idx_bk  Index of the Bank to configure
  * @param[in] GPIO     Pin number
  * @retval None
  */
void CreateMask(uint16_t idx_bk, uint8_t GPIO)
{
    switch(GPIO_to_SW_Conf[GPIO].IO_Channel)
        {
            case 1:
                BankMask[idx_bk].ch1 |= (uint8_t)(1 << GPIO_to_SW_Conf[GPIO].IO_Group); // Mask for all first channel
                break;
            case 2:
                BankMask[idx_bk].ch2 |= (uint8_t)(1 << GPIO_to_SW_Conf[GPIO].IO_Group); // Mask for all second channel
                break;
            case 3:
                BankMask[idx_bk].ch3 |= (uint8_t)(1 << GPIO_to_SW_Conf[GPIO].IO_Group); // Mask fo all third channel
                break;
            case 4:
                BankMask[idx_bk].ch4 |= (uint8_t)(1 << GPIO_to_SW_Conf[GPIO].IO_Group); // Mask for all fourth channel
                break;
            default:
                break;
        }
}


/**
  * @brief  Initializes the acquisition module.
  * @param  None
  * @retval retval
  */
TSL_Status_enum_T TSL_acq_Init(void)
{
    TSL_Init_GPIOs();
    TSL_Init_TIMs();
    TSL_Init_RI();
    return TSL_STATUS_OK;
}


/**
  * @brief Configures a Bank.
  * @param[in] idx_bk  Index of the Bank to configure
  * @retval Status
  */
TSL_Status_enum_T TSL_acq_BankConfig(TSL_tIndex_T idx_bk)
{
    uint8_t idx;
    uint16_t idx_dest;
    uint16_t idx_ch;
    uint16_t number_of_channels = 0;
    CONST TSL_Bank_T *p_bank; // Pointer to the current bank
    CONST TSL_ChannelDest_T *p_chDest; // Pointer to the first destination channel of the current bank
    CONST TSL_ChannelSrc_T *p_chSrc; // Pointer to the fisrt source channel of the current bank

    // Check parameters (if USE_FULL_ASSERT is defined)
    assert_param(IS_BANK_INDEX_OK(idx_bk));

    OldStatus = 0;

    TSL_Globals.This_Bank = idx_bk;
    p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
    number_of_channels = p_bank->NbChannels;
    p_chDest = p_bank->p_chDest;
    p_chSrc = p_bank->p_chSrc;

    // Reset the disable mask
    DisableSampling = 0;
    for (idx = 0; idx < MAX_CHANNEL_NUMBER_BY_GROUP; idx++)
        {
            DisableMask[idx] = 0;
        }

#if (TSLPRM_USE_SHIELD > 0)
    DISABLE_SAMPLING(p_bank->shield_sampling);
#endif

    // Loop for each channel of this bank
    for (idx_ch = 0; idx_ch < number_of_channels; idx_ch++)
        {

            idx_dest = p_chDest->IdxDest;

            // Mode Status OFF
            if (p_bank->p_chData[idx_dest].Flags.ObjStatus == TSL_OBJ_STATUS_OFF)
                {
                    // Update Mask if channels are disabled
                    DISABLE_MASK(p_chSrc->channel);
                    DISABLE_MASK(p_chSrc->sampling);
                }

            // Mode Status BURST ONLY
            if (p_bank->p_chData[idx_dest].Flags.ObjStatus == TSL_OBJ_STATUS_BURST_ONLY)
                {
                    DISABLE_SAMPLING(p_chSrc->sampling);
                }

            tab_MeasurementCounter[GPIO_to_SW_Conf[p_chSrc->sampling].IO_Group] = 0;

            // Next channel
            p_chSrc++;
            p_chDest++;
        }

    // Get Mask for the current bank
    CurrentBank[0] = (uint8_t)(BankMask[idx_bk].ch1 &
                               (~DisableMask[0])); // Mask for all 1st channel are used by channels and sampling for this bank
    CurrentBank[1] = (uint8_t)(BankMask[idx_bk].ch2 &
                               (~DisableMask[1])); // Mask for all 2nd channel are used by channels and sampling for this bank
    CurrentBank[2] = (uint8_t)(BankMask[idx_bk].ch3 &
                               (~DisableMask[2])); // Mask for all 3rd channel are used by channels and sampling for this bank
    CurrentBank[3] = (uint8_t)(BankMask[idx_bk].ch4 &
                               (~DisableMask[3])); // Mask for all 4th channel are used by channels and sampling for this bank

    CurrentChannel = ChannelMask[idx_bk]; // Mask for channels
    CurrentSampling = (uint8_t)(3 << (2 * (SamplingMask[idx_bk] - 1))); // Mask for sampling
    ChannelSampling = SamplingMask[idx_bk]; // Mask for the channel used by sampling

    // Channel's state of the current bank
    BankDone = (uint8_t)(CurrentBank[ChannelSampling - 1] & (~DisableSampling));

    // Select the IO Input register corresponding to the channel sampling (to optimize the measurement)
    p_IOIRx = RI_IOIRx_Register[ChannelSampling - 1];

    // Select the IO Mask register corresponding to the channel sampling (to optimize the measurement)
    p_IOMRx = RI_IOMRx_Register[ChannelSampling - 1];

    return TSL_STATUS_OK;
}


/**
  * @brief Start acquisition on a previously configured bank
  * @param None
  * @retval None
  */
void TSL_acq_BankStartAcq(void)
{
#if (TSLPRM_IODEF > 0)

    CONST TSL_Bank_T *p_bank = &(TSL_Globals.Bank_Array[0]);
    CONST TSL_ChannelSrc_T *p_chSrc;
    TSL_tNb_T number_of_channels = 0;
    TSL_tIndex_T idx_bk;
    TSL_tIndex_T idx_ch;

    //============================
    // All GPIOs in Input floating
    //============================
    for (idx_bk = 0; idx_bk < TSLPRM_TOTAL_BANKS; idx_bk++)
        {
            p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
            p_chSrc = p_bank->p_chSrc;

#if (TSLPRM_USE_SHIELD > 0)
            // GPIO in Input
            GPIO_DDR_IN(p_bank->shield_sampling);
            GPIO_DDR_IN(p_bank->shield_channel);
            // GPIO in floating mode
            GPIO_CR1_FLOATING(p_bank->shield_sampling);
            GPIO_CR1_FLOATING(p_bank->shield_channel);
#endif // TSLPRM_USE_SHIELD

            number_of_channels = p_bank->NbChannels;

            for (idx_ch = 0;
                    idx_ch < number_of_channels;
                    idx_ch++)
                {
                    // GPIO in Input
                    GPIO_DDR_IN(p_chSrc->sampling);
                    GPIO_DDR_IN(p_chSrc->channel);
                    // GPIO in floating mode
                    GPIO_CR1_FLOATING(p_chSrc->sampling);
                    GPIO_CR1_FLOATING(p_chSrc->channel);
                    p_chSrc++;
                }
        }
#endif // TSLPRM_IODEF

    // Test if this bank is not empty
    if (BankDone != 0)
        {

            // Set the AL bit to exit from WFI mode only on PXS interrupt
            CFG->GCR |= (uint8_t)CFG_GCR_AL;

            //--------------------------------------------
            // Configure Timer3 for the MaxCount detection
            //--------------------------------------------

            // Clear the Slave timer counter
            TIM3->CNTRH = 0;
            TIM3->CNTRL = 0;

            // Timer3 interruption routine to detect MaxCount
            // Warning: the high byte must be written before the low byte
            TIM3->CCR2H = (uint8_t)((TSL_Params.AcqMax+1) >> 8);
            TIM3->CCR2L = (uint8_t)(TSL_Params.AcqMax+1);

            // Clear all Timer3 flags...
            TIM3->SR1 = 0;
            TIM3->SR2 = 0;

            // Enable Capture/Compare 2 interrupt: MaxCount
            TIM3->IER |= 0x04; // CC2IE=1

            //--------------------------------------------

            // Enable necessary IOs
            RI->IOCMR1 |= (uint8_t)CurrentBank[0];
            RI->IOCMR2 |= (uint8_t)CurrentBank[1];
            RI->IOCMR3 |= (uint8_t)CurrentBank[2];
            RI->IOCMR4 |= (uint8_t)CurrentBank[3];

            // Discharge all capacitors
            RI->IOSR1 &= (uint8_t)(~CurrentBank[0]);
            RI->IOSR2 &= (uint8_t)(~CurrentBank[1]);
            RI->IOSR3 &= (uint8_t)(~CurrentBank[2]);
            RI->IOSR4 &= (uint8_t)(~CurrentBank[3]);

            // Wait a complete discharge
            SoftDelay(TSLPRM_DELAY_DISCHARGE_ALL);

            // Configure channel capacitors and sampling capacitors
            RI->IOGCR = (uint8_t)(0x55 & (~CurrentSampling));

            RI->IOSR1 |= (uint8_t)CurrentBank[0];
            RI->IOSR2 |= (uint8_t)CurrentBank[1];
            RI->IOSR3 |= (uint8_t)CurrentBank[2];
            RI->IOSR4 |= (uint8_t)CurrentBank[3];

            // Start acquisition
            TSL_Acq_Status = TSL_STATUS_BUSY;

            // Start the Master timer counter
            TIM2->CR1 |= 0x01; // CEN=1
        }
    else
        {
            TSL_Acq_Status = TSL_STATUS_OK;
        }
}


/**
  * @brief Wait end of acquisition
  * @param None
  * @retval status
  */
TSL_Status_enum_T TSL_acq_BankWaitEOC(void)
{
    return TSL_Acq_Status;
}


/**
  * @brief Return the current measure
  * @param[in] index Index of the measure source
  * @retval Measure
  */
TSL_tMeas_T TSL_acq_GetMeas(TSL_tIndex_T index)
{
    return(tab_MeasurementCounter[index]);
}


/**
  * @brief  Check noise (not used)
  * @param  None
  * @retval Status
  */
TSL_AcqStatus_enum_T TSL_acq_CheckNoise(void)
{
    return TSL_ACQ_STATUS_OK;
}


/**
  * @brief Used during HW acquisition mode.
  * @param  None
  * @retval None
  * @note Must be called by the TIM3 Capture/Compare interrupt routine.
  */
void TSL_CT_HWacq_TIM3(void)
{
    uint8_t new_status;
    uint8_t idx = 0;
    uint16_t timer_count;

    TIM2->CR1 &= (uint8_t)(~0x01); // Stop master counter

    RI->IOMR1 = 0;
    RI->IOMR2 = 0;
    RI->IOMR3 = 0;
    RI->IOMR4 = 0;

    // Discharge all capacitors (electrode and sampling capacitor IOs)
    RI->IOSR1 &= (uint8_t)(~(CurrentBank[0]));
    RI->IOSR2 &= (uint8_t)(~(CurrentBank[1]));
    RI->IOSR3 &= (uint8_t)(~(CurrentBank[2]));
    RI->IOSR4 &= (uint8_t)(~(CurrentBank[3]));

    TSL_Acq_Status = TSL_STATUS_OK;

    // Clear all Timer3 flags...
    TIM3->SR1 = 0;
    TIM3->SR2 = 0;

    // Read capture counter
    timer_count = (uint16_t)(TIM3->CCR1H << 8);
    timer_count += TIM3->CCR1L;

    new_status = (uint8_t)(BankDone & (~(OldStatus)));

    while ((new_status != 0) && (idx < 8))
        {
            if ((new_status & (1 << idx)) != 0)
                {
                    tab_MeasurementCounter[idx] = timer_count;
                    new_status &= (uint8_t)(~(1 << idx));
                    OldStatus |= (uint8_t)(1 << idx);
                    *p_IOMRx |= (uint8_t)(1 << idx); // Mask IO which reach VIH
                }
            idx++;
        }
}


/**
  * @brief Used during HW acquisition mode.
  * @param  None
  * @retval None
  * @note Must be called by the RI interrupt routine.
  *       Timer 2 and 3 are halted during this interrupt but counter is not reset.
  */
void TSL_CT_HWacq_RI(void)
{
    CONST TSL_Bank_T *p_bank = &(TSL_Globals.Bank_Array[0]);
    CONST TSL_ChannelSrc_T *p_chSrc;
    TSL_tNb_T number_of_channels = 0;
    TSL_tIndex_T idx_bk;
    TSL_tIndex_T idx_ch;

    __IO uint8_t IOIRx;
    uint8_t new_status;
    uint8_t idx = 0;
    uint16_t timer_count;

    IOIRx = *p_IOIRx;

    // Test RI Input register corresponding to sampling capacitors
    if ((IOIRx & BankDone) != OldStatus)
        {
            // Read capture counter
            timer_count = (uint16_t)(TIM3->CCR1H << 8);
            timer_count += TIM3->CCR1L;

            new_status = (uint8_t)((BankDone & IOIRx) & (~(OldStatus)));

            while ((new_status != 0) && (idx < 8))
                {
                    if ((new_status & (1 << idx)) != 0)
                        {
                            tab_MeasurementCounter[idx] = timer_count;
                            new_status &= (uint8_t)(~(1 << idx));
                            OldStatus |= (uint8_t)(1 << idx);
                            *p_IOMRx |= (uint8_t)(1 << idx); // Mask IO which reach VIH
                        }
                    idx++;
                }

            // When Current bank is completed
            if ((OldStatus == BankDone))
                {

                    // Disable master counter
                    TIM2->CR1 &= (uint8_t)(~0x01); // Stop master counter

                    // Reset IO Mask
                    RI->IOMR1 = 0;
                    RI->IOMR2 = 0;
                    RI->IOMR3 = 0;
                    RI->IOMR4 = 0;

                    // Disable necessary IOs
                    RI->IOSR1 &= (uint8_t)(~(CurrentBank[0]));
                    RI->IOSR2 &= (uint8_t)(~(CurrentBank[1]));
                    RI->IOSR3 &= (uint8_t)(~(CurrentBank[2]));
                    RI->IOSR4 &= (uint8_t)(~(CurrentBank[3]));

                    RI->IOCMR1 &= (uint8_t)(~(CurrentBank[0]));
                    RI->IOCMR2 &= (uint8_t)(~(CurrentBank[1]));
                    RI->IOCMR3 &= (uint8_t)(~(CurrentBank[2]));
                    RI->IOCMR4 &= (uint8_t)(~(CurrentBank[3]));

                    for (idx_bk = 0; idx_bk < TSLPRM_TOTAL_BANKS; idx_bk++)
                        {
                            p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
                            p_chSrc = p_bank->p_chSrc;

                            number_of_channels = p_bank->NbChannels;

#if (TSLPRM_USE_SHIELD > 0)
                            // GPIO in Output
                            GPIO_DDR_OUT(p_bank->shield_sampling);
                            GPIO_DDR_OUT(p_bank->shield_channel);
                            // GPIO in PP
                            GPIO_CR1_PP(p_bank->shield_sampling);
                            GPIO_CR1_PP(p_bank->shield_channel);
                            // Output in Low level
                            GPIO_ODR_LOW(p_bank->shield_sampling);
                            GPIO_ODR_LOW(p_bank->shield_channel);
#endif
                            // Initialize the mask for channel and sampling
                            for (idx_ch = 0; idx_ch < number_of_channels; idx_ch++)
                                {
                                    // GPIO are configured in PP Low mode when inactive
                                    // GPIO in Output
                                    GPIO_DDR_OUT(p_chSrc->sampling);
                                    GPIO_DDR_OUT(p_chSrc->channel);
                                    // GPIO in PP
                                    GPIO_CR1_PP(p_chSrc->sampling);
                                    GPIO_CR1_PP(p_chSrc->channel);
                                    // Output in Low level
                                    GPIO_ODR_LOW(p_chSrc->sampling);
                                    GPIO_ODR_LOW(p_chSrc->channel);
                                    // Next channel
                                    p_chSrc++;
                                }
                        }

#if TSLPRM_USE_ZONE > 0

                    TSL_acq_BankGetResult(TSL_Globals.This_Bank, 0, 0); // Get Bank Result

                    if ((TSL_Globals.This_Zone == 0) || (TSL_Globals.Index_In_This_Zone >= TSL_Globals.This_Zone->NbBanks))
                        {
                            CFG->GCR &= (uint8_t)(~CFG_GCR_AL); // Reset Activation level to resume main processing
                            TSL_Globals.This_Bank = 0;
                        }
                    else
                        {
                            if (TSL_acq_ZoneConfig(TSL_Globals.This_Zone, TSL_Globals.Index_In_This_Zone) != TSL_STATUS_ERROR)
                                {
                                    // Start Bank acquisition
                                    TSL_acq_BankStartAcq();
                                }
                            else
                                {
                                    CFG->GCR &= (uint8_t)(~CFG_GCR_AL); // Reset Activation level to resume main processing
                                    TSL_Globals.This_Bank = 0;
                                }
                        }
#else
                    CFG->GCR &= (uint8_t)(~CFG_GCR_AL);
#endif
                }
        }

    // Reset Interrupt flag
    RI->CR |= 0x02; // CAIF=1
    TSL_Acq_Status = TSL_STATUS_OK;
}


/**
  * @brief Check if a filter must be used on the current channel (not used)
  * @param[in] pCh Pointer on the channel data information
  * @retval Result TRUE if a filter can be applied
  */
TSL_Bool_enum_T TSL_acq_UseFilter(TSL_ChannelData_T *pCh)
{
    return TSL_TRUE;
}


/**
  * @brief Compute the Delta value
  * @param[in] ref Reference value
  * @param[in] meas Last Measurement value
  * @retval Delta value
  */
TSL_tDelta_T TSL_acq_ComputeDelta(TSL_tRef_T ref, TSL_tMeas_T meas)
{
    return((TSL_tDelta_T)(ref - meas));
}


/**
  * @brief Compute the Measurement value
  * @param[in] ref Reference value
  * @param[in] delta Delta value
  * @retval Measurement value
  */
TSL_tMeas_T TSL_acq_ComputeMeas(TSL_tRef_T ref, TSL_tDelta_T delta)
{
    return((TSL_tMeas_T)(ref - delta));
}


/**
  * @brief Test if the Reference is incorrect (not used)
  * @param[in] pCh Pointer on the channel data information
  * @retval Result TRUE if the Reference is out of range
  */
TSL_Bool_enum_T TSL_acq_TestReferenceOutOfRange(TSL_ChannelData_T *pCh)
{
    return TSL_FALSE;
}


/**
  * @brief Test if the measure has crossed the reference target (not used)
  * @param[in] pCh Pointer on the channel data information
  * @param[in] new_meas Measure of the last acquisition on this channel
  * @retval Result TRUE if the Reference is valid
  */
TSL_Bool_enum_T TSL_acq_TestFirstReferenceIsValid(TSL_ChannelData_T *pCh, TSL_tMeas_T new_meas)
{
    return TSL_TRUE;
}


#if defined(__ICCSTM8__)
    #pragma optimize=low
#endif
/**
  * @brief  Software delay (private routine)
  * @param  val Wait delay
  * @retval None
  */
void SoftDelay(uint16_t val)
{
    uint16_t idx;
    for (idx = val; idx > 0; idx--)
        {
            nop();
        }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
