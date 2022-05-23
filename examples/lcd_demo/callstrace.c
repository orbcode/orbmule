#include "soc_header.h"
#define TRACE_CHANNEL (30)
//#define DELAY for (uint32_t d=0; d<100000; d++) asm volatile ("NOP")
#define DELAY {}


/* These routines (or analogues of them) are generally provided by CMSIS, but defining them here makes things more independent */
__attribute__ ((no_instrument_function)) static uint32_t _internal_get_PRIMASK(void)
{
  uint32_t result;

  __ASM volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}

__attribute__ ((no_instrument_function)) static void _internal_set_PRIMASK(uint32_t priMask)

{
  __ASM volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}

__attribute__ ((no_instrument_function)) static void _internal_disable_irq(void)
{
  __ASM volatile ("cpsid i" : : : "memory");
}

__attribute__ ((no_instrument_function))
              void __cyg_profile_func_enter (void *this_fn, void *call_site)
{

    if (!(ITM->TER&(1<<TRACE_CHANNEL))) return;
    uint32_t oldIntStat=_internal_get_PRIMASK();

    // This is not atomic, but by using the stack for
    //storing oldIntStat it doesn't matter
    _internal_disable_irq();
    while (ITM->PORT[TRACE_CHANNEL].u32 == 0);

    // This is CYCCNT - number of cycles of the CPU clock
    ITM->PORT[TRACE_CHANNEL].u32 = ((*((uint32_t *)0xE0001004))&0x03FFFFFF)|0x40000000;
    while (ITM->PORT[TRACE_CHANNEL].u32 == 0);
    ITM->PORT[TRACE_CHANNEL].u32 = (uint32_t)(call_site)&0xFFFFFFFE;
    while (ITM->PORT[TRACE_CHANNEL].u32 == 0);

    ITM->PORT[TRACE_CHANNEL].u32 = (uint32_t)this_fn&0xFFFFFFFE;
    DELAY;

    _internal_set_PRIMASK(oldIntStat);
}

__attribute__ ((no_instrument_function))
              void __cyg_profile_func_exit (void *this_fn, void *call_site)
{

    if (!(ITM->TER&(1<<TRACE_CHANNEL))) return;
    uint32_t oldIntStat=_internal_get_PRIMASK();
    _internal_disable_irq();
    while (ITM->PORT[TRACE_CHANNEL].u32 == 0);
    ITM->PORT[TRACE_CHANNEL].u32 = ((*((uint32_t *)0xE0001004))&0x03FFFFFF)|0x50000000;
    while (ITM->PORT[TRACE_CHANNEL].u32 == 0);
    ITM->PORT[TRACE_CHANNEL].u32 = (uint32_t)(call_site)&0xFFFFFFFE;
    while (ITM->PORT[TRACE_CHANNEL].u32 == 0);
    ITM->PORT[TRACE_CHANNEL].u32 = (uint32_t)this_fn&0xFFFFFFFE;
    DELAY;
     _internal_set_PRIMASK(oldIntStat);
}
