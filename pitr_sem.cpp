#include "predef.h" 
#include <ctype.h>
#include <basictypes.h>
#include <system.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <sim.h>
#include <bsp.h>
#include <cfinter.h>
#include "pitr_sem.h"



// Instruct the C++ compiler not to mangle the function name 
extern "C" 
{
void SetIntc0( long func, int vector, int level, int prio );
}

volatile DWORD Pit_Count;


static WORD pit_pcsr_clr;
static OS_SEM * pSem;
INTERRUPT( my_pitr_func, 0x2600 )
{
sim.pit[1].pcsr = pit_pcsr_clr;
Pit_Count++;
if(pSem) OSSemPost(pSem);
}

               
extern DWORD CPU_CLOCK;


void PiterSem(OS_SEM *p_toSem, int pit_per_sec)
{
pSem=p_toSem;

DWORD div=2;
DWORD pcsr=0x00F;
 while (((CPU_CLOCK/div)/pit_per_sec) > 65536) {div*=2; pcsr+=0x100;}


pit_pcsr_clr=pcsr; 
sim.pit[1].pmr = ((CPU_CLOCK/div)/pit_per_sec); // Set the PIT modulus value
sim.pit[1].pcsr =pcsr;


SetIntc0( ( long ) &my_pitr_func, 56, 2 /* IRQ 2 */, 3 );

}


