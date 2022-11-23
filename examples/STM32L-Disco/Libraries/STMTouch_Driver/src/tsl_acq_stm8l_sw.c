/**
  ******************************************************************************
  * @file    tsl_acq_stm8l_sw.c
  * @author  MCD Application Team
  * @version V1.4.4
  * @date    31-March-2014
  * @brief   This file contains all functions to manage the acquisition
  *          on STM8L products using the software acquisition mode.
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
#include "tsl_acq_stm8l_sw.h"
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

/** Define if maximum channel number is 3 or 4 according to the Device Density
  */
#if defined(STM8L15X_LD) || defined(STM8L10X)
    #define MAX_CHANNEL_NUMBER_BY_GROUP (4)
#else
    #define MAX_CHANNEL_NUMBER_BY_GROUP (3)
#endif // defined(STM8L15X_LD) || defined(STM8L10X)

#if defined(_COSMIC_)
    #define INLINE @inline
#elif defined(_RAISONANCE_)
    #define INLINE inline
#elif defined(_IAR_)
    #define INLINE
#else
    #error "Compiler not Supported"
#endif

/* Private macros ------------------------------------------------------------*/

#if !defined(STM8L10X)
    #define GPIO_PORT(GPIO)  (GPIO >> 3)   /**< Get the GPIO port*/
    #define GPIO_BIT(GPIO)   (GPIO & 0x07) /**< Get the GPIO pin number*/
#else
    #define GPIO_PORT(GPIO)  (GPIO >> 2)   /**< Get the GPIO port*/
    #define GPIO_BIT(GPIO)   (GPIO & 0x03) /**< Get the GPIO pin number*/
#endif // !defined(STM8L10X)

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

uint8_t SpreadCounter = TSLPRM_SPREAD_MIN;

uint16_t ChargeTransferCounter;  // This variable count the charge transfer number in the acquisition loop
uint8_t BankDone;                // Control if all activate sampling reach the VIH level
uint8_t CurrentSampling;         // Mask to control IOGCR register
uint8_t CurrentChannel;          // Mask to control IOGCR register
uint8_t ChannelSampling;         // Contain the channel number where all sampling are connected
uint8_t DisableSampling;         // Disable sampling mask when the Burst Only mode is activated for one channel of the current bank(not get the measure)

TSL_Bank_Config_Mask_T
BankMask[TSLPRM_TOTAL_BANKS]; // Complete masks (channel and sampling) to configure IOCMRx and IOSRx registers for all banks
uint8_t SamplingMask[TSLPRM_TOTAL_BANKS];            // Sampling mask to configure IOGCR register for all banks
uint8_t DisableMask[MAX_CHANNEL_NUMBER_BY_GROUP];    // Complete disable mask(channel and sampling) when the Channel OFF mode is activated for one channel of the current bank(to modifie the CurrentBank)
uint8_t CurrentBank[MAX_CHANNEL_NUMBER_BY_GROUP];    // Complete mask for the current bank

#if !defined(STM8L10X)

#if defined(STM8L15X_LD)
__IO uint8_t *RI_IOIRx_Register[MAX_CHANNEL_NUMBER_BY_GROUP] = {&(RI->IOIR1), &(RI->IOIR2), &(RI->IOIR3), &(RI->IOIR4)};
#else
__IO uint8_t *RI_IOIRx_Register[MAX_CHANNEL_NUMBER_BY_GROUP] = {&(RI->IOIR1), &(RI->IOIR2), &(RI->IOIR3)};
#endif // STM8L15X_LD

__IO uint8_t *p_IOIRx; // Pointer to the IOIRx register (x from 1 to 4)
GPIO_TypeDef *p_GPIOx[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF};

uint16_t tab_MeasurementCounter[8] = {0}; // Measurement of each sampling of the current bank
uint8_t ChannelMask[TSLPRM_TOTAL_BANKS];  // Channel mask to configure IOGCR register for all banks

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

#else // STM8L10X

__IO uint8_t *p_GPIOx_IDR;
__IO uint8_t *GPIOx_IDR[2] = {&(GPIOB->IDR), &(GPIOD->IDR)};

GPIO_TypeDef *p_GPIOx[] = {GPIOB, GPIOD};

uint16_t tab_MeasurementCounter[2] = {0};         // Measurement of each sampling of the current bank
uint8_t Bank_IO_CompMask[TSLPRM_TOTAL_BANKS];     // IO Mask for Comparator register to control SW
uint8_t BankSamplingCompMask[TSLPRM_TOTAL_BANKS]; // Sampling Mask for Comparator register to control SW
uint8_t Bank_IOShield_CompMask[TSLPRM_TOTAL_BANKS];
uint8_t BankSamplingShieldCompMask[TSLPRM_TOTAL_BANKS];

/* Table which do the link between GPIO and switch configuation:{x,y}
   x = channel number
   y = group number - 1
   Note: {0,0} = not connect to IO switch
*/
CONST TSL_IOConf_T GPIO_to_SW_Conf[8] =
{
    // Port B definitions
    {1, 0}, // PB0 is channel 1 of Group 1
    {2, 0}, // PB1 is channel 2 of Group 1
    {1, 1}, // PB2 is channel 1 of Group 2
    {2, 1}, // PB3 is channel 2 of Group 2
    // Port D definitions
    {3, 0}, // PD0 is channel 3 of Group 1
    {4, 0}, // PD1 is channel 4 of Group 1
    {3, 1}, // PD2 is channel 3 of Group 2
    {4, 1}  // PD3 is channel 4 of Group 2
};

#endif // !defined(STM8L10X)

/* Private functions prototype -----------------------------------------------*/
void SoftDelay(uint16_t val);
void CreateMask(TSL_tIndex_T idx_bk, uint8_t GPIO);
void GetCounter(__IO uint8_t *p_reg, uint8_t *p_old_status);
INLINE void __Delay_Charge(void);
void CreateIOMask(TSL_tIndex_T idx_bk, uint8_t GPIO);
void CreateSamplingMask(TSL_tIndex_T idx_bk, uint8_t GPIO);
#if (TSLPRM_USE_SPREAD_SPECTRUM > 0)
    INLINE void SwSpreadSpectrum(void);
#endif


/**
  * @brief  Delay in NOPs to apply during charging time.
  * @param  None
  * @retval None
  */
INLINE void __Delay_Charge(void)
{
#if TSLPRM_DELAY_CHARGE > 0
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 1
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 2
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 3
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 4
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 5
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 6
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 7
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 8
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 9
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 10
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 11
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 12
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 13
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 14
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 15
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 16
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 17
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 18
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 19
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 20
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 21
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 22
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 23
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 24
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 25
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 26
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 27
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 28
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 29
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 30
    nop();
#endif
#if TSLPRM_DELAY_CHARGE > 31
    nop();
#endif
}


/**
  * @brief  Delay in NOPs to apply during transfering time.
  * @param  None
  * @retval None
  */
INLINE void __Delay_Transfer(void)
{
#if TSLPRM_DELAY_TRANSFER > 0
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 1
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 2
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 3
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 4
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 5
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 6
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 7
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 8
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 9
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 10
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 11
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 12
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 13
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 14
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 15
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 16
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 17
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 18
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 19
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 20
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 21
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 22
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 23
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 24
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 25
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 26
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 27
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 28
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 29
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 30
    nop();
#endif
#if TSLPRM_DELAY_TRANSFER > 31
    nop();
#endif
}


/**
  * @brief  Initialize the acquisition module.
  * @param  None
  * @retval Status
  */
TSL_Status_enum_T TSL_acq_Init(void)
{
    CONST TSL_Bank_T *p_bank = &(TSL_Globals.Bank_Array[0]); // Pointer to the first bank
    CONST TSL_ChannelSrc_T *p_chSrc = p_bank->p_chSrc; // Pointer to the source channel of the current bank
    TSL_tNb_T number_of_channels = 0;
    TSL_tIndex_T idx_bk;
    TSL_tIndex_T idx_ch;

#if !defined(STM8L10X)
    // Enable comparator clock to activate the RI block
    CLK->PCKENR2 |= CLK_PCKENR2_COMP;
#endif // !defined(STM8L10X)

    // Enable mode software (bit AM)
#if defined(STM8L15X_LD)
    RI->CR &= (uint8_t)(~0x04); // Mode SW
#endif // STM8L15X_LD

    // Initializes each bank and configures the used GPIO
    for (idx_bk = 0; idx_bk < TSLPRM_TOTAL_BANKS; idx_bk++)
        {
            p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
            p_chSrc = p_bank->p_chSrc;
            number_of_channels = p_bank->NbChannels;

#if !defined(STM8L10X)
            // Mask Initialization
            BankMask[idx_bk].ch1 = 0;
            BankMask[idx_bk].ch2 = 0;
            BankMask[idx_bk].ch3 = 0;
            BankMask[idx_bk].ch4 = 0;
#else
            // Mask Initialization
            BankMask[idx_bk].GPIOB_IO = 0;
            BankMask[idx_bk].GPIOB_Samp = 0;
            BankMask[idx_bk].GPIOD_IO = 0;
            BankMask[idx_bk].GPIOD_Samp = 0;
#endif // !defined(STM8L10X)

            // Get which channel is used for sampling only one time because it's the same for each couple
            SamplingMask[idx_bk] = (uint8_t)GPIO_to_SW_Conf[p_chSrc->sampling].IO_Channel;

#if (TSLPRM_USE_SHIELD > 0)

            // Create Mask per bank for shield
#if !defined(STM8L10X)
            CreateMask(idx_bk, p_bank->shield_sampling);
            CreateMask(idx_bk, p_bank->shield_channel);
            ChannelMask[idx_bk] |= (uint8_t)(3 << (2 * ((GPIO_to_SW_Conf[p_bank->shield_channel].IO_Channel) - 1)));
#else
            CreateIOMask(idx_bk, p_bank->shield_channel);
            CreateSamplingMask(idx_bk, p_bank->shield_sampling);
#endif // !defined(STM8L10X)

            // Check if shield sampling capacitors are in the same number of channel for each group
            if ((SamplingMask[idx_bk] != (uint8_t)GPIO_to_SW_Conf[p_bank->shield_sampling].IO_Channel))
                {
                    return TSL_STATUS_ERROR;
                }

            // GPIO in Output
            GPIO_DDR_OUT(p_bank->shield_sampling);
            GPIO_DDR_OUT(p_bank->shield_channel);
            // GPIO in PP
            GPIO_CR1_PP(p_bank->shield_sampling);
            GPIO_CR1_PP(p_bank->shield_channel);
            // Output in Low level
            GPIO_ODR_LOW(p_bank->shield_sampling);
            GPIO_ODR_LOW(p_bank->shield_channel);

            // Activate Comparator 1
            if (GPIO_to_SW_Conf[p_bank->shield_sampling].IO_Group == 0)
                {
                    COMP->CR |= 0x02;
                }
            // Activate Comparator 2
            if (GPIO_to_SW_Conf[p_bank->shield_sampling].IO_Group == 1)
                {
                    COMP->CR |= 0x04;
                }

#endif // TSLPRM_USE_SHIELD

            // Initialize the mask for channel and sampling
            for (idx_ch = 0; idx_ch < number_of_channels; idx_ch++)
                {
#if !defined(STM8L10X)
                    // Create Mask per bank for channel and sampling
                    CreateMask(idx_bk, p_chSrc->channel);
                    CreateMask(idx_bk, p_chSrc->sampling);
                    ChannelMask[idx_bk] |= (uint8_t)(3 << (2 * ((GPIO_to_SW_Conf[p_chSrc->channel].IO_Channel) - 1)));
                    // Check if sampling capacitors are in the same number of channel for each group
                    if ((SamplingMask[idx_bk] != (uint8_t)GPIO_to_SW_Conf[p_chSrc->sampling].IO_Channel))
                        {
                            return TSL_STATUS_ERROR;
                        }
#else
                    // Activate Comparator 1
                    if (p_chSrc->IdxSrc == 0)
                        {
                            COMP->CR |= 0x02;
                        }
                    // Activate Comparator 2
                    if (p_chSrc->IdxSrc == 1)
                        {
                            COMP->CR |= 0x04;
                        }
                    // Create Mask per bank for channel and sampling
                    CreateIOMask(idx_bk,p_chSrc->channel);
                    Bank_IO_CompMask[idx_bk] |= (uint8_t)(1 << (GPIO_to_SW_Conf[p_chSrc->channel].IO_Channel - 1));
                    Bank_IO_CompMask[idx_bk] = (uint8_t)(Bank_IO_CompMask[idx_bk] << (4 * GPIO_to_SW_Conf[p_chSrc->channel].IO_Group));
                    CreateSamplingMask(idx_bk,p_chSrc->sampling);
                    BankSamplingCompMask[idx_bk] |= (uint8_t)(1 << (GPIO_to_SW_Conf[p_chSrc->sampling].IO_Channel - 1));
                    BankSamplingCompMask[idx_bk] = (uint8_t)(BankSamplingCompMask[idx_bk] << (4 *
                                                   GPIO_to_SW_Conf[p_chSrc->sampling].IO_Group));
                    if ((SamplingMask[idx_bk] != (uint8_t)GPIO_to_SW_Conf[p_chSrc->sampling].IO_Channel))
                        {
                            return TSL_STATUS_ERROR;
                        }
#if (TSLPRM_USE_SHIELD > 0)
                    Bank_IOShield_CompMask[idx_bk] |= (uint8_t)(1 << (GPIO_to_SW_Conf[p_bank->shield_channel].IO_Channel - 1));
                    Bank_IOShield_CompMask[idx_bk] = (uint8_t)(Bank_IOShield_CompMask[idx_bk] << (4 *
                                                     GPIO_to_SW_Conf[p_bank->shield_channel].IO_Group));
                    BankSamplingShieldCompMask[idx_bk] |= (uint8_t)(1 << (GPIO_to_SW_Conf[p_bank->shield_sampling].IO_Channel - 1));
                    BankSamplingShieldCompMask[idx_bk] = (uint8_t)(BankSamplingShieldCompMask[idx_bk] <<
                                                         (4 * GPIO_to_SW_Conf[p_bank->shield_sampling].IO_Group));
                    Bank_IO_CompMask[idx_bk] = (uint8_t)(Bank_IO_CompMask[idx_bk] | Bank_IOShield_CompMask[idx_bk]);
                    BankSamplingCompMask[idx_bk] = (uint8_t)(BankSamplingCompMask[idx_bk] | BankSamplingShieldCompMask[idx_bk]);
#endif
#endif // !defined(STM8L10X)

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

                    p_chSrc++; // Next channel
                }

#if !defined(STM8L10X)
            // Unlock IO to RI register: IO controlled by GPIO
            RI->IOCMR1 &= (uint8_t)(~BankMask[idx_bk].ch1);
            RI->IOCMR2 &= (uint8_t)(~BankMask[idx_bk].ch2);
            RI->IOCMR3 &= (uint8_t)(~BankMask[idx_bk].ch3);
#if defined(STM8L15X_LD)
            RI->IOCMR4 &= (uint8_t)(~BankMask[idx_bk].ch4);
#endif // STM8L15X_LD || STM8L10X
#endif // !defined(STM8L10X)
        }

    return  TSL_STATUS_OK;
}


#if !defined(STM8L10X)
/**
  * @brief  Create Mask for all banks
  * @param[in] idx_bk  Index of the Bank to configure
  * @param[in] GPIO    Pin number
  * @retval None
  */
void CreateMask(TSL_tIndex_T idx_bk, uint8_t GPIO)
{
    switch (GPIO_to_SW_Conf[GPIO].IO_Channel)
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
#if defined(STM8L15X_LD) || defined(STM8L10X)
            case 4:
                BankMask[idx_bk].ch4 |= (uint8_t)(1 << GPIO_to_SW_Conf[GPIO].IO_Group); // Mask for all fourth channel
                break;
#endif // STM8L15X_LD || STM8L10X
            default:
                break;
        }
}

#else

/**
  * @brief  Create IO Mask for all banks
  * @param[in] idx_bk  Index of the Bank to configure
  * @param[in] GPIO    Pin number
  * @retval None
  */
void CreateIOMask(TSL_tIndex_T idx_bk, uint8_t GPIO)
{
    switch (GPIO_PORT(GPIO))
        {
            case 0:
                BankMask[idx_bk].GPIOB_IO |= (uint8_t)(1 << GPIO_BIT(GPIO));
                break;
            case 1:
                BankMask[idx_bk].GPIOD_IO |= (uint8_t)(1 << GPIO_BIT(GPIO));
                break;
            default:
                break;
        }
}


/**
  * @brief  Create Sampling Mask for all banks
  * @param[in] idx_bk  Index of the Bank to configure
  * @param[in] GPIO    Pin number
  * @retval None
  */
void CreateSamplingMask(TSL_tIndex_T idx_bk, uint8_t GPIO)
{
    switch (GPIO_PORT(GPIO))
        {
            case 0:
                BankMask[idx_bk].GPIOB_Samp |= (uint8_t)(1 << GPIO_BIT(GPIO));
                break;
            case 1:
                BankMask[idx_bk].GPIOD_Samp |= (uint8_t)(1 << GPIO_BIT(GPIO));
                break;
            default:
                break;
        }
}

#endif // !defined(STM8L10X)


#if (TSLPRM_USE_SPREAD_SPECTRUM > 0)
/**
  * @brief  Spread Spectrum using a variable software delay.
  * @param  None
  * @retval None
  */
INLINE void SwSpreadSpectrum(void)
{
    uint8_t idx;

    SpreadCounter++;

    if (SpreadCounter == TSLPRM_SPREAD_MAX)
        {
            SpreadCounter = TSLPRM_SPREAD_MIN;
        }

    idx = SpreadCounter;

    while (--idx) {}
}
#endif


/**
  * @brief  Bank configuration
  * @param[in] idx_bk  Index of the Bank to configure
  * @retval Status
  */
TSL_Status_enum_T TSL_acq_BankConfig(TSL_tIndex_T idx_bk)
{
    uint8_t idx_i;
#if defined(STM8L10X)
    uint8_t GroupUsed = 0;
#endif
    TSL_tIndex_T idx_dest;
    TSL_tIndex_T idx_ch;
    TSL_tNb_T number_of_channels = 0;
    CONST TSL_Bank_T *p_bank; // Pointer to the current bank
    CONST TSL_ChannelDest_T *p_chDest; // Pointer to the first destination channel of the current bank
    CONST TSL_ChannelSrc_T *p_chSrc; // Pointer to the fisrt source channel of the current bank

    // Check parameters (if USE_FULL_ASSERT is defined)
    assert_param(IS_BANK_INDEX_OK(idx_bk));

    p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
    number_of_channels = p_bank->NbChannels;
    p_chDest = p_bank->p_chDest;
    p_chSrc = p_bank->p_chSrc;

    // Reset the disable mask
    DisableSampling = 0;
    for (idx_i = 0; idx_i < MAX_CHANNEL_NUMBER_BY_GROUP; idx_i++)
        {
            DisableMask[idx_i] = 0;
        }

    BankDone = 0;

#if (TSLPRM_USE_SHIELD > 0)
    DISABLE_SAMPLING(p_bank->shield_sampling);
#endif // TSLPRM_USE_SHIELD

    ChannelSampling = SamplingMask[idx_bk]; // Mask for the channel used by sampling

    // Loop for each channel of this bank
    for (idx_ch = 0; idx_ch < number_of_channels; idx_ch++)
        {
            idx_dest = p_chDest->IdxDest;
#if defined(STM8L10X)
            if (p_chSrc->IdxSrc == 0)
                {
                    GroupUsed |= 0x01;
                }
            if (p_chSrc->IdxSrc == 1)
                {
                    GroupUsed |= 0x02;
                }
#endif // defined(STM8L10X)

            // Mode Status OFF
            if (p_bank->p_chData[idx_dest].Flags.ObjStatus == TSL_OBJ_STATUS_OFF)
                {
#if !defined(STM8L10X)
                    // Update Mask if channels are disabled
                    DISABLE_MASK(p_chSrc->channel);
                    DISABLE_MASK(p_chSrc->sampling);
#else
                    // Update Mask if channels are disabled
                    if (GPIO_to_SW_Conf[p_chSrc->channel].IO_Channel > 2)
                        {
                            DisableMask[2] |= (uint8_t)(1 << ((2 * GPIO_to_SW_Conf[p_chSrc->channel].IO_Group) +
                                                              (GPIO_to_SW_Conf[p_chSrc->channel].IO_Channel - 3)));
                        }
                    else
                        {
                            DisableMask[0] |= (uint8_t)(1 << ((2 * GPIO_to_SW_Conf[p_chSrc->channel].IO_Group) +
                                                              (GPIO_to_SW_Conf[p_chSrc->channel].IO_Channel - 1)));
                        }
                    if (GPIO_to_SW_Conf[p_chSrc->sampling].IO_Channel > 2)
                        {
                            DisableMask[3] |= (uint8_t)(1 << ((2 * GPIO_to_SW_Conf[p_chSrc->sampling].IO_Group) +
                                                              (GPIO_to_SW_Conf[p_chSrc->sampling].IO_Channel - 3)));
                        }
                    else
                        {
                            DisableMask[1] |= (uint8_t)(1<<((2 * GPIO_to_SW_Conf[p_chSrc->sampling].IO_Group) +
                                                            (GPIO_to_SW_Conf[p_chSrc->sampling].IO_Channel - 1)));
                        }
#endif // !defined(STM8L10X)
                }

            // Mode Status BURST ONLY
            if (p_bank->p_chData[idx_dest].Flags.ObjStatus == TSL_OBJ_STATUS_BURST_ONLY)
                {
#if !defined(STM8L10X)
                    DISABLE_SAMPLING(p_chSrc->sampling);
#else
                    if (p_chSrc->IdxSrc == 0)
                        {
                            GroupUsed &= (uint8_t)(~0x01);
                        }
                    if (p_chSrc->IdxSrc == 1)
                        {
                            GroupUsed &= (uint8_t)(~0x02);
                        }
#endif // !defined(STM8L10X)
                }

            tab_MeasurementCounter[GPIO_to_SW_Conf[p_chSrc->sampling].IO_Group] = 0;

            // Next channel
            p_chSrc++;
            p_chDest++;
        }

#if !defined(STM8L10X)
    //Get Mask for the current bank
    CurrentBank[0] = (uint8_t)(BankMask[idx_bk].ch1 &
                               (~DisableMask[0])); // Mask for all 1st channel are used by channels and sampling for this bank
    CurrentBank[1] = (uint8_t)(BankMask[idx_bk].ch2 &
                               (~DisableMask[1])); // Mask for all 2nd channel are used by channels and sampling for this bank
    CurrentBank[2] = (uint8_t)(BankMask[idx_bk].ch3 &
                               (~DisableMask[2])); // Mask for all 3rd channel are used by channels and sampling for this bank
#if defined(STM8L15X_LD)
    CurrentBank[3] = (uint8_t)(BankMask[idx_bk].ch4 &
                               (~DisableMask[3])); // Mask for all 4th channel are used by channels and sampling for this bank
#endif // STM8L15X_LD
    CurrentChannel = ChannelMask[idx_bk];  // Mask for channels
    CurrentSampling = (uint8_t)(3 << (2 * (SamplingMask[idx_bk] - 1))); // Mask for sampling

    // Channel's state of the current bank
    BankDone = (uint8_t)(CurrentBank[ChannelSampling - 1] & (~DisableSampling));

    // Select the Input register corresponding to the channel sampling (to optimize the measurement)
    p_IOIRx = RI_IOIRx_Register[ChannelSampling - 1];

#else
    //Get Mask for the current bank
    CurrentBank[0] = (uint8_t)(BankMask[idx_bk].GPIOB_IO & (~DisableMask[0]));
    CurrentBank[1] = (uint8_t)(BankMask[idx_bk].GPIOB_Samp & (~DisableMask[1]));
    CurrentBank[2] = (uint8_t)(BankMask[idx_bk].GPIOD_IO & (~DisableMask[2]));
    CurrentBank[3] = (uint8_t)(BankMask[idx_bk].GPIOD_Samp & (~DisableMask[3]));

    CurrentChannel = (uint8_t)(Bank_IO_CompMask[idx_bk]); // Mask for channels
    CurrentSampling = (uint8_t)(BankSamplingCompMask[idx_bk]); // Mask for sampling

    // Select the Input register corresponding to the channel sampling (to optimize the measurement) and update BankDone, which is the mask where there are sampling capacitors
    if (ChannelSampling > 2)  // GPIOD
        {
            p_GPIOx_IDR = GPIOx_IDR[1];
            if ((GroupUsed & 0x01) == 1)
                {
                    BankDone |= (uint8_t)(1 << (ChannelSampling - 3));
                }
            if((GroupUsed & 0x02) == 2)
                {
                    BankDone |= (uint8_t)(1 << (2 + (ChannelSampling - 3)));
                }

        }
    else // GPIOB
        {
            p_GPIOx_IDR = GPIOx_IDR[0];
            if ((GroupUsed & 0x01) == 1)
                {
                    BankDone |= (uint8_t)(1 << (ChannelSampling - 1));
                }
            if ((GroupUsed & 0x02) == 2)
                {
                    BankDone |= (uint8_t)(1 << (2 + (ChannelSampling - 1)));
                }
        }

#endif // !defined(STM8L10X)

    return TSL_STATUS_OK;
}


#if !defined(STM8L10X)

/**
  * @brief  Start acquisition
  * @param  None
  * @retval None
  */
void TSL_acq_BankStartAcq(void)
{
    CONST TSL_Bank_T *p_bank = &(TSL_Globals.Bank_Array[0]);
    CONST TSL_ChannelSrc_T *p_chSrc;
    TSL_tNb_T number_of_channels = 0;
    TSL_tIndex_T idx_bk;
    TSL_tIndex_T idx_ch;
    uint8_t step3, step5, deadtime1, deadtime2; //intermediate variables to speed-up the acquisition loop

    uint8_t old_status = 0;

    ChargeTransferCounter = 0;

#if (TSLPRM_IODEF > 0)
    //============================
    // All GPIOs in Input floating
    //============================
    for (idx_bk = 0; idx_bk < TSLPRM_TOTAL_BANKS; idx_bk++)
        {
            p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
            p_chSrc = p_bank->p_chSrc;

#if (TSLPRM_USE_SHIELD > 0)
            // GPIO in floating mode
            GPIO_CR1_FLOATING(p_bank->shield_sampling);
            GPIO_CR1_FLOATING(p_bank->shield_channel);
            // GPIO in Input
            GPIO_DDR_IN(p_bank->shield_sampling);
            GPIO_DDR_IN(p_bank->shield_channel);
#endif // TSLPRM_USE_SHIELD

            number_of_channels = p_bank->NbChannels;

            for (idx_ch = 0;
                    idx_ch < number_of_channels;
                    idx_ch++)
                {
                    // GPIO in floating mode
                    GPIO_CR1_FLOATING(p_chSrc->sampling);
                    GPIO_CR1_FLOATING(p_chSrc->channel);
                    // GPIO in Input
                    GPIO_DDR_IN(p_chSrc->sampling);
                    GPIO_DDR_IN(p_chSrc->channel);
                    p_chSrc++;
                }
        }
#endif // TSLPRM_IODEF

    // Test if this bank is not empty
    if (BankDone != 0)
        {
            // Enable necessary IOs
            RI->IOCMR1 |= (uint8_t)CurrentBank[0];
            RI->IOCMR2 |= (uint8_t)CurrentBank[1];
            RI->IOCMR3 |= (uint8_t)CurrentBank[2];
#if defined(STM8L15X_LD)
            RI->IOCMR4 |= (uint8_t)CurrentBank[3];
#endif // STM8L15X_LD

            RI->IOSR1 |= (uint8_t)CurrentBank[0];
            RI->IOSR2 |= (uint8_t)CurrentBank[1];
            RI->IOSR3 |= (uint8_t)CurrentBank[2];
#if defined(STM8L15X_LD)
            RI->IOSR4 |= (uint8_t)CurrentBank[3];
#endif // STM8L15X_LD

            /* STEP1 : Discharging all capacitors
            ==> all IOs in Push-Pull LOW */
            RI->IOGCR &= (uint8_t)(~(CurrentChannel | CurrentSampling));

            /* STEP2: Waiting for complete discharge */
            SoftDelay(TSLPRM_DELAY_DISCHARGE_ALL);
            // Dead Time
            RI->IOGCR |= (uint8_t)(0xAA & (CurrentChannel | CurrentSampling));

            // Close switch sampling
            RI->IOGCR |= CurrentSampling;

            /* Copmpute RI->IOGCR for each step */
            /* STEP3: Charging C-Touch
            ==> Channels in Push-Pull HIGH
            ==> Sampling kept open */
            step3 = (uint8_t)(RI->IOGCR ^ CurrentChannel);
            /* Deadtime */
            deadtime1 = RI->IOGCR ; // equivalent to step3 ^ (uint8_t)CurrentChannel;
            /* STEP5: Transfering C-Touch charge in C-Sampling
            ==> Close IOs Switchs */
            step5 = (uint8_t)(RI->IOGCR | CurrentChannel);
            /* Deadtime */
            deadtime2 = (uint8_t)(step5 & (0xAA | (~CurrentChannel)));

            // Loop while all sampling have not reach the VIH level
            do
                {
                    /* STEP3: Charging C-Touch */
                    RI->IOGCR = step3;
                    // Get the measurement of counter if the value of Input register change
                    if ((*p_IOIRx & BankDone) != old_status)
                        {
                            GetCounter(p_IOIRx, &old_status);
                        }

                    /* STEP4 : Waiting for good chargement */
                    __Delay_Charge();

#if (TSLPRM_USE_SPREAD_SPECTRUM > 0)
                    SwSpreadSpectrum();
#endif

                    /* Dead Time */
                    RI->IOGCR = deadtime1;
                    /* STEP5: Transfering C-Touch charge in C-Sampling */
                    RI->IOGCR = step5;

                    ChargeTransferCounter++;

                    /* STEP6: Waiting for good transfer */
                    __Delay_Transfer();

                    /* Dead Time  */
                    RI->IOGCR = deadtime1;
                }
            while ((old_status != BankDone) && (ChargeTransferCounter <= TSL_Params.AcqMax));

            // Get the value of counter if he reach the Max count
            if(ChargeTransferCounter > TSL_Params.AcqMax)
                {
                    GetCounter(&BankDone, &old_status);
                }

            // Disable necessary IOs
            RI->IOSR1 &= (uint8_t)(~(CurrentBank[0]));
            RI->IOSR2 &= (uint8_t)(~(CurrentBank[1]));
            RI->IOSR3 &= (uint8_t)(~(CurrentBank[2]));
#if defined(STM8L15X_LD)
            RI->IOSR4 &= (uint8_t)(~(CurrentBank[3]));
#endif

            RI->IOCMR1 &= (uint8_t)(~(CurrentBank[0]));
            RI->IOCMR2 &= (uint8_t)(~(CurrentBank[1]));
            RI->IOCMR3 &= (uint8_t)(~(CurrentBank[2]));
#if defined(STM8L15X_LD)
            RI->IOCMR4 &= (uint8_t)(~(CurrentBank[3]));
#endif

            //====================
            // All GPIOs in PP Low
            //====================
            for (idx_bk = 0; idx_bk < TSLPRM_TOTAL_BANKS; idx_bk++)
                {
                    p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
                    p_chSrc = p_bank->p_chSrc;

#if (TSLPRM_USE_SHIELD > 0)
                    // Output in Low level
                    GPIO_ODR_LOW(p_bank->shield_sampling);
                    GPIO_ODR_LOW(p_bank->shield_channel);
                    // GPIO in Output
                    GPIO_DDR_OUT(p_bank->shield_sampling);
                    GPIO_DDR_OUT(p_bank->shield_channel);
                    // GPIO in PP
                    GPIO_CR1_PP(p_bank->shield_sampling);
                    GPIO_CR1_PP(p_bank->shield_channel);
#endif // TSLPRM_USE_SHIELD

                    number_of_channels = p_bank->NbChannels;

                    for (idx_ch = 0;
                            idx_ch < number_of_channels;
                            idx_ch++)
                        {
                            // Output in Low level
                            GPIO_ODR_LOW(p_chSrc->sampling);
                            GPIO_ODR_LOW(p_chSrc->channel);
                            // GPIO in Output
                            GPIO_DDR_OUT(p_chSrc->sampling);
                            GPIO_DDR_OUT(p_chSrc->channel);
                            // GPIO in PP
                            GPIO_CR1_PP(p_chSrc->sampling);
                            GPIO_CR1_PP(p_chSrc->channel);
                            p_chSrc++;
                        }
                }

        }
}

#else // STM8L10X

/**
  * @brief  Start acquisition
  * @param  None
  * @retval None
  */
void TSL_acq_BankStartAcq(void)
{
    CONST TSL_Bank_T *p_bank = &(TSL_Globals.Bank_Array[0]);
    CONST TSL_ChannelSrc_T *p_chSrc;
    TSL_tNb_T number_of_channels = 0;
    TSL_tIndex_T idx_bk;
    TSL_tIndex_T idx_ch;

    uint8_t old_status = 0;

    ChargeTransferCounter = 0;

#if (TSLPRM_IODEF > 0)
    //============================
    // All GPIOs in Input floating
    //============================
    for (idx_bk = 0; idx_bk < TSLPRM_TOTAL_BANKS; idx_bk++)
        {
            p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
            p_chSrc = p_bank->p_chSrc;

#if (TSLPRM_USE_SHIELD > 0)
            // GPIO in floating mode
            GPIO_CR1_FLOATING(p_bank->shield_sampling);
            GPIO_CR1_FLOATING(p_bank->shield_channel);
            // GPIO in Input
            GPIO_DDR_IN(p_bank->shield_sampling);
            GPIO_DDR_IN(p_bank->shield_channel);
#endif

            number_of_channels = p_bank->NbChannels;

            for (idx_ch = 0;
                    idx_ch < number_of_channels;
                    idx_ch++)
                {
                    // GPIO in floating mode
                    GPIO_CR1_FLOATING(p_chSrc->sampling);
                    GPIO_CR1_FLOATING(p_chSrc->channel);
                    // GPIO in Input
                    GPIO_DDR_IN(p_chSrc->sampling);
                    GPIO_DDR_IN(p_chSrc->channel);

                    p_chSrc++;
                }
        }
#endif // TSLPRM_IODEF

    // Test if this bank is not empty
    if (BankDone != 0)
        {

#ifdef TSLPRM_PROTECT_IO_ACCESS
            disableInterrupts();
#endif //TSLPRM_PROTECT_IO_ACCESS

            /* STEP1 : Discharging all capacitors
            ==> all IOs in open-drain LOW */
            GPIOB->ODR &= (uint8_t)(~(CurrentBank[0] | CurrentBank[1]));
            GPIOB->CR1 &= (uint8_t)(~(CurrentBank[0] | CurrentBank[1]));
            GPIOB->DDR |= (uint8_t)(CurrentBank[0] | CurrentBank[1]);
            GPIOD->ODR &= (uint8_t)(~(CurrentBank[2] | CurrentBank[3]));
            GPIOD->CR1 &= (uint8_t)(~(CurrentBank[2] | CurrentBank[3]));
            GPIOD->DDR |= (uint8_t)(CurrentBank[2] | CurrentBank[3]);

#ifdef TSLPRM_PROTECT_IO_ACCESS
            enableInterrupts();
#endif // TSLPRM_PROTECT_IO_ACCESS

            COMP->CCS &= (uint8_t)(~(CurrentSampling |CurrentChannel));

            /* STEP2: Waiting for complete discharge */
            SoftDelay(TSLPRM_DELAY_DISCHARGE_ALL);

#ifdef TSLPRM_PROTECT_IO_ACCESS
            disableInterrupts();
#endif // TSLPRM_PROTECT_IO_ACCESS

            // Dead Time
            GPIOB->DDR &= (uint8_t)(~(CurrentBank[0] | CurrentBank[1]));
            GPIOD->DDR &= (uint8_t)(~(CurrentBank[2] | CurrentBank[3]));

#ifdef TSLPRM_PROTECT_IO_ACCESS
            enableInterrupts();
#endif // TSLPRM_PROTECT_IO_ACCESS

            GPIOB->ODR |= (uint8_t)(CurrentBank[0]);
            GPIOD->ODR |= (uint8_t)(CurrentBank[2]);

            // Loop while all sampling have not reach the VIH level
            do
                {

#ifdef TSLPRM_PROTECT_IO_ACCESS
                    disableInterrupts();
#endif // TSLPRM_PROTECT_IO_ACCESS

                    /* STEP3: Charging C-Touch
                    ==> Channels in Push-Pull HIGH
                    ==> Sampling kept open */
                    GPIOB->DDR |= (uint8_t)(CurrentBank[0]);
                    GPIOB->CR1 |= (uint8_t)(CurrentBank[0]);
                    GPIOD->DDR |= (uint8_t)(CurrentBank[2]);
                    GPIOD->CR1 |= (uint8_t)(CurrentBank[2]);

#ifdef TSLPRM_PROTECT_IO_ACCESS
                    enableInterrupts();
#endif // TSLPRM_PROTECT_IO_ACCESS

                    /* STEP4 : Waiting for good chargement */
                    __Delay_Charge();

#if (TSLPRM_USE_SPREAD_SPECTRUM > 0)
                    SwSpreadSpectrum();
#endif

#ifdef TSLPRM_PROTECT_IO_ACCESS
                    disableInterrupts();
#endif  // TSLPRM_PROTECT_IO_ACCESS

                    // Dead Time
                    GPIOB->CR1 &= (uint8_t)(~(CurrentBank[0]));
                    GPIOB->DDR &= (uint8_t)(~(CurrentBank[0]));
                    GPIOD->CR1 &= (uint8_t)(~(CurrentBank[2]));
                    GPIOD->DDR &= (uint8_t)(~(CurrentBank[2]));

#ifdef TSLPRM_PROTECT_IO_ACCESS
                    enableInterrupts();
#endif // TSLPRM_PROTECT_IO_ACCESS        

                    /* STEP5: Transfering C-Touch charge in C-Sampling
                    ==> Close IOs Switchs */

                    // Close switch sampling
                    COMP->CCS |= (uint8_t)CurrentSampling;
                    COMP->CCS |= (uint8_t)CurrentChannel;

                    /* STEP6: Waiting for good transfer */
                    __Delay_Transfer();

                    //Dead Time
                    COMP->CCS &= (uint8_t)(~(CurrentChannel | CurrentSampling));

#ifdef TSLPRM_PROTECT_IO_ACCESS
                    disableInterrupts();
#endif // TSLPRM_PROTECT_IO_ACCESS

                    // Get the measurement of counter if the value of Input register change
                    if ((*p_GPIOx_IDR & BankDone) != old_status)
                        {
                            GetCounter(p_GPIOx_IDR, &old_status);
                        }

#ifdef TSLPRM_PROTECT_IO_ACCESS
                    enableInterrupts();
#endif // TSLPRM_PROTECT_IO_ACCESS

                    ChargeTransferCounter++;

                }
            while ((old_status != BankDone) && (ChargeTransferCounter != (TSL_Params.AcqMax+1)));

            // Get the value of counter if he reach the Max count
            if(ChargeTransferCounter == (TSL_Params.AcqMax+1))
                {
                    GetCounter(&BankDone, &old_status);
                }

            //====================
            // All GPIOs in PP Low
            //====================
            for (idx_bk = 0; idx_bk < TSLPRM_TOTAL_BANKS; idx_bk++)
                {
                    p_bank = &(TSL_Globals.Bank_Array[idx_bk]);
                    p_chSrc = p_bank->p_chSrc;

#if (TSLPRM_USE_SHIELD > 0)
                    // Output in Low level
                    GPIO_ODR_LOW(p_bank->shield_sampling);
                    GPIO_ODR_LOW(p_bank->shield_channel);
                    // GPIO in Output
                    GPIO_DDR_OUT(p_bank->shield_sampling);
                    GPIO_DDR_OUT(p_bank->shield_channel);
                    // GPIO in PP
                    GPIO_CR1_PP(p_bank->shield_sampling);
                    GPIO_CR1_PP(p_bank->shield_channel);
#endif // TSLPRM_USE_SHIELD

                    number_of_channels = p_bank->NbChannels;

                    for (idx_ch = 0;
                            idx_ch < number_of_channels;
                            idx_ch++)
                        {
                            // Output in Low level
                            GPIO_ODR_LOW(p_chSrc->sampling);
                            GPIO_ODR_LOW(p_chSrc->channel);
                            // GPIO in Output
                            GPIO_DDR_OUT(p_chSrc->sampling);
                            GPIO_DDR_OUT(p_chSrc->channel);
                            // GPIO in PP
                            GPIO_CR1_PP(p_chSrc->sampling);
                            GPIO_CR1_PP(p_chSrc->channel);
                            p_chSrc++;
                        }
                }

        }
}

#endif


/**
  * @brief  Do the measurement
  * @param  *p_reg Pointer to the Input register
  * @param  *p_old_status Pointer to the previous status value
  * @retval None
  */

void GetCounter(__IO uint8_t *p_reg, uint8_t *p_old_status)
{

    uint8_t new_status = 0;
    uint8_t idx_i = 0;
    uint8_t mask_i = 1;
#if defined(STM8L10X)
    uint8_t idx_j = 4;
    uint8_t idx_group = 0;
#else
    uint8_t idx_j = 8;
#endif // defined(STM8L10X)

    new_status = *p_reg;
    new_status =  (uint8_t)(new_status & BankDone & (~(*p_old_status)));

    while ((new_status != 0) && (idx_i < idx_j))
        {
            if ((new_status & mask_i) != 0)
                {
#if defined(STM8L10X)
                    tab_MeasurementCounter[idx_group]= ChargeTransferCounter;
#else
                    tab_MeasurementCounter[idx_i] = ChargeTransferCounter;
#endif // defined(STM8L10X)
                    *p_old_status |= mask_i;
                    new_status &= (uint8_t)(~mask_i);
                }
            idx_i++;
            mask_i <<= 1;
#if defined(STM8L10X)
            if (idx_i > 1)
                {
                    idx_group = 1;
                }
#endif // defined(STM8L10X)
        }
}


/**
  * @brief Wait end of acquisition
  * @param None
  * @retval status
  */
TSL_Status_enum_T TSL_acq_BankWaitEOC(void)
{
    return TSL_STATUS_OK;
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
