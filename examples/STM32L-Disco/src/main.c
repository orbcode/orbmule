#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "itm_messages.h"

extern void SystemCoreClockUpdate(void);
extern uint32_t SystemCoreClock;

#define N 10000
#define EOL "\r\n"

#define OVERVIEW_CHANNEL 3
#define RESULTS_CHANNEL  2
#define SIEVE_CHANNEL    1
#define ALPHABET_CHANNEL 0

#define TICKS_PER_SEC (1000)
volatile uint32_t ms;

// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// Internal routines
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
void SysTick_Handler(void)

{
    ms++;
}
// ====================================================================================================
static char notprime[N+1];

//  Eratosthenes sieve

static void _sieve(void)

{
    int k;
    ITM_SendString( SIEVE_CHANNEL,"Sieve run starts" EOL);

    for (int y=0; y<=N; y++)
        {
            notprime[y]=0;
        }

    k=2;
    while(k<=N)
        {
            int t=2;
            while(t*k<=N)
                {
                    notprime[t*k]=1;
                    t++;
                }
            k++;
            while(k<=N && notprime[k]==1)
                {
                    k++;
                }
        }
    ITM_SendString( SIEVE_CHANNEL,"Sieve run ends" EOL);
}
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// Externally available routines
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
int main( void )

{
    uint32_t iterations;
    uint32_t ems;
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / TICKS_PER_SEC );
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    ITM_Start();
    ITM_ChannelEnable(SIEVE_CHANNEL);
    //ITM_ChannelEnable(ALPHABET_CHANNEL);
    ITM_ChannelEnable(RESULTS_CHANNEL);
    ITM_ChannelEnable(OVERVIEW_CHANNEL);
    ITM_Enable();
    
    while (1)
        {
            ITM_SendString( OVERVIEW_CHANNEL,"\n\nSimple Example running" EOL);
            iterations=0;
            ems = ms+10*TICKS_PER_SEC;
            while (ms<ems)
                {
                    _sieve();
                    for (char a='A'; a<='Z'; a++)
                        {
                            ITM_Send8(ALPHABET_CHANNEL,a);
                        }
                    ITM_SendString(ALPHABET_CHANNEL,EOL);
                    iterations++;
                }
            ITM_Send32(RESULTS_CHANNEL,iterations);
            ITM_SendString( OVERVIEW_CHANNEL,"Run ends\r\n" EOL);
            while (1);
        }

    /* should not reach this statement */
    return 0;
}
// ====================================================================================================
