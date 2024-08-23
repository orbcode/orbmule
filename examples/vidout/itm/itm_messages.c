#include <stdarg.h>
#include <stdio.h>
#include "itm_messages.h"

#define MAX_STRLEN 80

/* Define registers locally in case CMSIS isn't being used */
#define DBG_DEMCR (*(volatile uint32_t *)0xE000EDFC)
#define DBG_TCR (*(volatile uint32_t *)0xE0000E80)
#define DBG_TER (*(volatile uint32_t *)0xE0000E00)
#define DBG_PORT ((volatile uint32_t *)0xE0000000)
#define DBG_LAR (*(volatile uint32_t *)0xE0000FB0)

#define DBG_DEMCR_TRCENA (1 << 24)
#define DBG_TCR_ITMENA (1 << 0)
// ====================================================================================================
static inline uint32_t _sendITM(uint32_t ch, uint32_t d, uint8_t size) {
   __asm__ volatile ("cpsid i" : : : "memory");
  if ((DBG_DEMCR & DBG_DEMCR_TRCENA) && /* Trace enabled */
      (DBG_TCR & DBG_TCR_ITMENA) &&     /* ITM enabled */
      (ITM_ChannelEnabled(ch))          /* ITM Port c enabled */
  ) {
        while (!DBG_PORT[0]) { __asm__("nop;");
        }; /* Port available? */
    switch (size) {
    case 1:
      (*((volatile uint8_t *)&(DBG_PORT[ch]))) = (uint8_t)d;
      break;
    case 2:
      (*((volatile uint16_t *)&(DBG_PORT[ch]))) = (uint16_t)d;
      break;
    case 4:
      DBG_PORT[ch] = d;
      break;
    default:
      size=0;
      break;
    }
  }
   __asm__ volatile ("cpsie i" : : : "memory");
  return (size);
}
// ====================================================================================================
uint32_t ITM_Send8(uint32_t c, uint8_t d)

{
  return _sendITM(c, d, 1);
}
// ====================================================================================================
uint32_t ITM_Send16(uint32_t c, uint16_t d)

{
  return _sendITM(c, d, 2);
}
// ====================================================================================================
inline uint32_t ITM_Send32(uint32_t c, uint32_t d)

{
  return _sendITM(c, d, 4);
}
// ====================================================================================================
uint32_t ITM_SendString(uint32_t c, char *s)

{
  uint32_t cc = 0;

  if ((s) && (ITM_ChannelEnabled(c))) {
    while ((s) && (*s)) {
      _sendITM(c, *s++, 1);
      cc++;
    }
  }
  return cc;
}
// ====================================================================================================
uint32_t ITM_Write(uint32_t c, const void *d, uint32_t l)

{
  const char *s = (const char *)d;

  if (s && (ITM_ChannelEnabled(c))) {
    while (l > 3) {
      {
        l -= _sendITM(c, *(uint32_t *)s, 4);
        s += 4;
      }
    }

    if (l > 1) {
      l -= _sendITM(c, *(uint16_t *)s, 2);
      s += 2;
    }

    if (l) {
      _sendITM(c, *s, 1);
    }
  }
  return s - (char *)d;
}
// ====================================================================================================
int ITM_Printf( uint32_t c, const char *fmt, ... )

// Print to output stream.
  
{
    static char op[MAX_STRLEN];
    int r;

    va_list va;
    va_start( va, fmt );
    r = vsnprintf( op, MAX_STRLEN, fmt, va );
    va_end( va );
    ITM_Write( c, op, r );
    return r;
}
// ====================================================================================================
void ITM_Enable(void)

{
  DBG_DEMCR |= DBG_DEMCR_TRCENA;
  DBG_TER = 0;
}
// ====================================================================================================
void ITM_Disable(void)

{
  DBG_DEMCR &= ~DBG_DEMCR_TRCENA;
}
// ====================================================================================================
void ITM_ChannelEnable(uint32_t ch)

{
  DBG_TER |= (1 << ch);
}
// ====================================================================================================
void ITM_ChannelDisable(uint32_t ch)

{
  DBG_TER &= ~(1 << ch);
}
// ====================================================================================================
bool ITM_ChannelEnabled(uint32_t ch)

{
  return ((DBG_TER & (1 << ch)) != 0);
}
// ====================================================================================================
