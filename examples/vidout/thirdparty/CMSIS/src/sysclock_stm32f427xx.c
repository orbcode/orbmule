#include "stm32f427xx.h"

#define PLL_M 6   // PLLM = 6
#define PLL_N 168 // PLLN = 168
#define PLL_P 0   // PLLP = 2

#define AHB_PR (0 << 4)   // 1
#define APB1_PR (5 << 10) // 4
#define APB2_PR (4 << 13) // 2

void SystemClock_Config(void)

/* This routine with thanks to
   https://controllerstech.com/stm32-clock-setup-using-registers/ */
/* to save the weight of the HAL, and 'cos I can't be bothered to write it
   myself.            */
{
  /* Enable HSE and wait for it to spin up */
  RCC->CR |= 1 << 16;
  while (!(RCC->CR & (1 << 17)))
    ;

  /* Set Power enable clock and Voltage regulator */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  PWR->CR |= PWR_CR_VOS_1;

  /* Enable latency and caches */
  FLASH->ACR = (1 << 8) | (1 << 9) | (1 << 10) | (5 << 0);

  /* Setup prescalars */
  RCC->CFGR = (RCC->CFGR & (0x0f << 4)) | AHB_PR;
  RCC->CFGR = (RCC->CFGR & (0x7 << 10)) | APB1_PR;
  RCC->CFGR = (RCC->CFGR & (0x07 << 13)) | APB2_PR;

  /* Set main PLL */
  RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | (1 << 22);

  /* Enable PLL and wait for it to be ready */
  RCC->CR |= (1 << 24);
  while (!(RCC->CR & (1 << 25)))
    ;

  /* cutover */
  RCC->CFGR |= (2 << 0);
  while (!(RCC->CFGR & (2 << 2)))
    ;
}
