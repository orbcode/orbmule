/* Simple accessors for MTB functionality without any requirement for specific primitives support (e.g. CMSIS) */

#define REG_MTB_POSITION           ((volatile unsigned int *)(0x41006000))
#define   MTB_POSITION_POINTER(x) ((x)&0xfffffff0)
#define   MTB_POSITION_WRAP (1<<2)

#define REG_MTB_MASTER             ((volatile unsigned int *)(0x41006004))
#define   MTB_MASTER_EN (1<<31)
#define   MTB_MASTER_HALTREQ (1<<9)
#define   MTB_MASTER_RAMPRIV (1<<8)
#define   MTB_MASTER_SFRWPRIV (1<<7)
#define   MTB_MASTER_TSTOPEN  (1<<6)
#define   MTB_MASTER_TSTARTEN (1<<5)
#define   MTB_MASTER_MASK(x)  ((x)&0x1f)

#define REG_MTB_FLOW               ((volatile unsigned int *)(0x41006008))
#define   MTB_FLOW_AUTOSTOP (1<<0)
#define   MTB_FLOW_AUTOHALT (1<<1)
#define   MTB_FLOW_WMARK(x) ((x)&0xfffffff0)

#define REG_MTB_BASE               ((volatile unsigned int *)(0x4100600C))

// ====================================================================================================
// ====================================================================================================
// ====================================================================================================

void mtb_disable(void)
{
  *REG_MTB_MASTER &= ~MTB_MASTER_EN;
}

// ====================================================================================================

int mtb_enable(int mtb_size)

{
  mtb_disable();
  if ((mtb_size < 16) || (__builtin_popcount(mtb_size) != 1))
    {
    // MTB must be at least 16 bytes and be a power of 2
    return 0;
    }

    // scrub MTB SRAM so it's easy to see what has gotten written to
  for (int i=0; i<mtb_size/4; i++) REG_MTB_BASE[i]=0;
  const int mask = __builtin_ctz(mtb_size) - 4;

  *REG_MTB_POSITION = 0;
  *REG_MTB_MASTER = MTB_MASTER_EN | MTB_MASTER_MASK(mask);
  return 1;
}

// ====================================================================================================

void *mtb_get(void)
{
  return (void*)*REG_MTB_BASE;
}

// ====================================================================================================

int mtb_length(void)
{
  return (2<< (MTB_MASTER_MASK(*REG_MTB_MASTER)+4) );
}

// ====================================================================================================
