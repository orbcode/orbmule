#include <stdio.h>
#include <stdlib.h>

#include "config.h"

// ====================================================================================================
// Setup routine called before main
// ====================================================================================================
void SystemInit(void)
{

}
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
#define N 10
void (*crashfunc)(void);
 
char notprime[N+1];

//  Eratosthenes sieve

static void _sieve(void)

{
  crashfunc = (void (*)(void))0x40000000;
    int k;

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
		    //		    if (k==N-100) crashfunc();
                }
        }
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
    while (1)
        {
            for (char a='A'; a<'Z'; a++)
                {
                    _sieve();
                }
        }

    /* should not reach this statement */
    return 0;
}
// ====================================================================================================