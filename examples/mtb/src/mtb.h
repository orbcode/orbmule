#ifndef _MTB_H_
#define _MTB_H_

/* Simple accessors for MTB functionality without any requirement for specific primitives support (e.g. CMSIS) */

// ====================================================================================================

void mtb_enable(int on);
int mtb_setup(int mtb_size);
void *mtb_get(void);
int mtb_length(void);

// ====================================================================================================

#endif
