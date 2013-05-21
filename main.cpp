#include "predef.h"
#include <basictypes.h>         // Include for variable types
#include <serialirq.h>         // Use UART interrupts instead of polling
#include <constants.h>          // Include for constands like MAIN_PRIO
#include <system.h>             // Include for system functions
#include <netif.h>
#include <ethernet_diag.h>
#include <autoupdate.h>
#include <pins.h>
#include <syslog.h>
#include <sim.h>
#include <cfinter.h>
#include <smarttrap.h>
#include "pitr_sem.h"

extern "C"
{
   void UserMain(void *pd); // prevent C++ name mangling
   void SetIntc0( long func, int vector, int level, int prio );
}



volatile BOOL bDoingAA;


const char * AppName = "SBL2e Minimum Build";

#define UART_STDIO (0)
#define UART_XBEE (1)
#define UART_LCD (2)



#define  LCD_CMD         (0xFE) //            'command prefix

#define  LCD_SPECCMD     (0x7C) //            'special command prefix
#define  LCD_CLS         (0x51) //            'Clear entire LCD screen
#define  LCD_HOME        (0x46) //            'Puts the cursor at 0, 0                        
#define  LCD_OFF         (0x08) //            'Display off
#define  LCD_ON          (0x0C) //            'Display ON
#define  LCD_NOCURS      (0x0C) //            'Make cursor invisible
#define  LCD_ULCURS      (0x0E) //            'Show underline cursor
#define  LCD_BLKCURS     (0x0D) //            'Show blinking block cursor

#define  LCD_CURPOS      (0x45) //            'set cursor  + position  

#define  LCD_SCRRIGHT    (0x1C) //            'scroll right
#define  LCD_SCRLEFT     (0x18) //            'scroll left
#define  LCD_RIGHT       (0x14) //            'Move cursor right 
#define  LCD_LEFT        (0x10) //            'Move cursor left

void LCD_Home()
{
writechar(UART_LCD,LCD_CMD);
writechar(UART_LCD,LCD_HOME);
}

void LCD_Cls()
{
writechar(UART_LCD,LCD_CMD);
writechar(UART_LCD,LCD_CLS);
}

void LCD_XY(BYTE x, BYTE y)
{
writechar(UART_LCD,LCD_CMD);
writechar(UART_LCD,LCD_CURPOS);
writechar(UART_LCD,(y*0x40)+x);
}


void LCD_Init()
{
writechar(UART_LCD,LCD_CMD);
writechar(UART_LCD,LCD_ON);
writechar(UART_LCD, LCD_SPECCMD);
writechar(UART_LCD, 157);
}


bool Switch16()
{
CPU_Pins[29].function(CPUPIN29_GPIO);
if(CPU_Pins[29].read()) return 1;
return 0;
}


const int CS_PIN[3] ={28,13,15};

#define PIN_DIN (25)
#define PIN_DOUT (26)
#define PIN_CLK (27)

void ExADInit()
{
for(int i=0; i<3; i++) 
	{
	 CPU_Pins[CS_PIN[i]].function(0);
	 CPU_Pins[CS_PIN[i]]=1;

	}

CPU_Pins[PIN_CLK].function(0); //Clk  CPUPIN27_QSPI_CLK 
CPU_Pins[PIN_DOUT].function(0); //DOUT CPUPIN26_QSPI_DOUT 
CPU_Pins[PIN_DIN].function(0); //DIN  CPUPIN25_QSPI_DIN 

CPU_Pins[PIN_CLK]=0;
CPU_Pins[PIN_DOUT]=0;
CPU_Pins[PIN_DIN].read();
}


void tdlay()
{
asm (" nop");
asm (" nop");
asm (" nop");
asm (" nop");

}


BYTE SR8(BYTE bo)
{
BYTE bin=0;
for(int i=0; i<8; i++)
{
if(bo & 0x80) 
	CPU_Pins[PIN_DOUT]=1;
else
	CPU_Pins[PIN_DOUT]=0;
	bo=(bo<<1);

	tdlay();
	CPU_Pins[PIN_CLK]=1;
	tdlay();
	if(CPU_Pins[PIN_DIN].read()) 
		bin=(bin<<1)|1;
	else
		bin=(bin<<1);
	CPU_Pins[PIN_CLK]=0;
	tdlay();
}

return bin;
}


WORD Sample(int cs, int ch)
{
CPU_Pins[PIN_CLK]=0;
CPU_Pins[PIN_DOUT]=0;
CPU_Pins[PIN_DIN].read();
CPU_Pins[CS_PIN[cs]]=0;

if(ch>3) 
		SR8(0x7); 
	else 
		SR8(0x06);

tdlay();
BYTE bmsb=SR8((BYTE)((ch & 3)<<6));
tdlay();
BYTE blsb=SR8(0);
CPU_Pins[CS_PIN[cs]]=1;
tdlay();

WORD w=(((WORD)bmsb<<8)|blsb) & 0xFFF;

if(w>0x800) return 1;
return 0;
}

volatile DWORD nADC;
DWORD ADSum[8];

DWORD Throttle;
DWORD Elevator;
DWORD Alieron;
DWORD Rudder;
DWORD N[4];
DWORD Switches;




INTERRUPT(AD_Done,0x2700)
{
sim.adc.adstat=0x1800;
if(bDoingAA)
{
	sim.adc.ctrl1=0;
	return ;
}

nADC++;
if((nADC & 0x1F)==0)
 {
  	 Rudder  =ADSum[0];
	 N[1]    =ADSum[1];
	 Throttle=ADSum[2];
	 N[0]    =ADSum[3];
	 N[3]    =ADSum[4];
	 Alieron =ADSum[5];
	 N[2]    =ADSum[6];
	 Elevator=ADSum[7];
  for(int i=0; i<8; i++)
	 ADSum[i]=sim.adc.adrslt[i]; 
  }
else
{
	for(int i=0; i<8; i++)
	ADSum[i]+=sim.adc.adrslt[i]; 
}
}


void EnableISRAD()
{
   //Clear the limits and offsets 
   for ( int i = 0; i < 8; i++ )
   {
      sim.adc.adllmt[i] = 0;
      sim.adc.adhlmt[i] = 0x3FFF;
      sim.adc.adofs[i] = 0;
   }

   SetIntc0( (long)&AD_Done, 49,2,1);


   sim.adc.ctrl1 = 0x4000; //Stop

   // Set clock value and parallel sampling
   sim.adc.ctrl2 = 0x1F; // Default value of clock_div is 7 


   sim.adc.adzcc = 0;    

   // Set up to sample all 8 channels
   sim.adc.adlst1 = 0x3210; //  x011 x010 x001 x000 
   sim.adc.adlst2 = 0x7654; //  x111 x110 x101 x100 


   sim.adc.adsdis = 0;   
   sim.adc.power = 0;    
   sim.adc.cal = 0;      

   sim.adc.ctrl1 = 0x0002;  // 0 0 0 0  0 0 0 0 0000 0 010 
   sim.adc.ctrl1 = 0x2802;  // 0 0 1 0  1 0 0 0 0000 0 010 

}

DWORD ReadSwitches()
{
DWORD tmp=0;
for(int i=0; i<8; i++)
	tmp=(tmp<<1)| Sample(1,i);

for(int i=0; i<8; i++)
	tmp=(tmp<<1)| Sample(2,i);
tmp^=0xFFC0;

if(Switch16()) tmp|=(0x10000);
return tmp;

}


int shutdownfunc()
{
	bDoingAA=true;
	return 1;
}
const DWORD Throttle_min =0x30000*2;
const DWORD Throttle_max =0x52000*2;

const DWORD Rudder_min =0x2E000*2;
const DWORD Rudder_max =0x51000*2;

const DWORD Elevator_min =0x2E800*2;
const DWORD Elevator_max =0x51000*2;


const DWORD Alieron_min =0x2E000*2;
const DWORD Alieron_max =0x4E000*2;

const DWORD NMins[4] ={0x02000*2,0x02000*2,0x02000*2,0x02000*2};
const DWORD NMaxs[4] ={0x7D000*2,0x7D000*2,0x7D000*2,0x7D000*2};
                                       


void writenib(int port,WORD w)
{
w=w&0x0f;

if(w>=10) writechar(port,('A'+w-10));
	else
		writechar(port,w+'0');

}




void writepot(int port, DWORD v, DWORD minv, DWORD maxv)
{
float val=v;
float maxf=maxv;
float minf=minv;
WORD w;

if(v<minv) w=1000;
else
if(v>maxv) w=0;
else
{

val-=maxf;
val=-1000.0*val/(maxf-minf);
if(val >1000.0) val=1000.0; 
if(val < 0.0) val=0; 
w=(WORD)(short)(val);
}

writenib(port,w>>8);
writenib(port,w>>4);
writenib(port,w);

}


void writeanalog(int port, DWORD v, DWORD minv, DWORD maxv)
{
float val=v;
float maxf=maxv;
float minf=minv;
WORD w;
if(v<minv) w=(WORD)(short)-1000;
else
if(v>maxv) w=1000;
else
{

val-=((maxf+minf)*0.5);
val=2000.0*val/(maxf-minf);
if(val >1000.0) val=1000.0; 
if(val <-1000.0) val=-1000.0; 
w=(WORD)(short)(val);
w=(WORD)val;
}

writenib(port,w>>8);
writenib(port,w>>4);
writenib(port,w);
}

BYTE Screen[32];
BYTE SentScreen[32];


void ProcessXbeeChar(BYTE c)
{
static BYTE lc;
if ((lc>=128) && (lc<(129+31)))
{
 Screen[lc-129]=c;
 //iprintf("[%c] at %ld\r\n",c,lc);
}
lc=c;
}


#define SM_TASK_STK (256)
#define SmOSSimpleTaskCreate(x,p) { static DWORD   func_##x_Stk[SM_TASK_STK] __attribute__( ( aligned( 4 ) ) ); OSTaskCreate(x,NULL,(void *)&func_##x_Stk[SM_TASK_STK],(void*)func_##x_Stk,p); }



void SyncScreen(void * p)
{
while(1)
	{static int pos;
if((Screen[pos]!=0))
{
 int x=(pos %16);
 int y=(pos/16);

	LCD_XY(x,y);
	writechar(UART_LCD,Screen[pos]);
	//iprintf("%d:%d:%c\r\n",x,y,Screen[pos]);
	SentScreen[pos]=Screen[pos];
  }
   pos++;
	if(pos==32) pos=0;
 OSTimeDly(1);
}
}

/*-------------------------------------------------------------------
 * UserMain
 *-----------------------------------------------------------------*/
void UserMain(void *pd)
{
    SimpleUart( 0, SystemBaud );   // initialize UART 0
    assign_stdio( 0 );             // use UART 0 for stdio

	SimpleUart( UART_XBEE, 115200 );   // initialize UART 0
	SimpleUart( UART_LCD, 9600 );   // initialize UART 0
 
	//iprintf("Wating...\r\n");
    //sgetchar(0);

	LCD_Init();
	OSTimeDly(10);
	LCD_Cls();

		writestring(UART_LCD,"NBCTL "__DATE__);

	LCD_XY(0,1);
	writestring(UART_LCD,"at "__TIME__);
    
	InitializeStack();
    {
        WORD ncounts = 0;
        while ( ( !bEtherLink ) && ( ncounts < 2*TICKS_PER_SECOND ) )
        {
            ncounts++;
            OSTimeDly( 1 );
        }
    }

    EnableAutoUpdate();
	EnableSmartTraps();
	update_shutdown_func=shutdownfunc;
    OSChangePrio( MAIN_PRIO ); // set standard UserMain task priority
	
   
	LCD_Home();

	ExADInit(); 

	CPU_Pins[61].function(1); //All the A/D pins in A/D mode...
	CPU_Pins[62].function(1);
	CPU_Pins[63].function(1);
	CPU_Pins[64].function(1);
	CPU_Pins[65].function(1);
	CPU_Pins[66].function(1);
	CPU_Pins[67].function(1);
	CPU_Pins[68].function(1);

	CPU_Pins[23].function(CPUPIN23_URXD1);
	CPU_Pins[24].function(CPUPIN24_UTXD1);


	EnableISRAD();

	iprintf("Application built on %s on %s\r\n", __TIME__, __DATE__ );
	writestring(UART_XBEE,"Application built on " __TIME__" on " __DATE__ "\r\n");

	

	DWORD LC=nADC;

	OS_SEM pit_sem;
	OSSemInit(&pit_sem,0);
	PiterSem(&pit_sem, 50);

    SmOSSimpleTaskCreate(SyncScreen,MAIN_PRIO+1);

	DWORD LastSec=Secs;
	DWORD LastSeq=0;
    while ( !bDoingAA )
    {DWORD tc=nADC;
	 static WORD Seq;

        OSSemPend(&pit_sem,0);
		
		while(charavail(UART_XBEE))
		{
		 ProcessXbeeChar(sgetchar(UART_XBEE));
		}
		writechar(UART_XBEE,'[');
	    writenib(UART_XBEE,Seq>>12);
		writenib(UART_XBEE,Seq>>8);
		writenib(UART_XBEE,Seq>>4);
		writenib(UART_XBEE,Seq++);
		writechar(UART_XBEE,',');
		writeanalog(UART_XBEE,Elevator,Elevator_min,Elevator_max);
		writechar(UART_XBEE,',');
		writeanalog(UART_XBEE,Alieron ,Alieron_min,Alieron_max);
		writechar(UART_XBEE,',');
	    writeanalog(UART_XBEE,Rudder  ,Rudder_min,Rudder_max);
		writechar(UART_XBEE,',');
		writeanalog(UART_XBEE,Throttle,Throttle_min,Throttle_max);

		for(int i=0; i<4; i++)
		{
         writechar(UART_XBEE,',');
		 writepot(UART_XBEE,N[i]  ,NMins[i],NMaxs[i]);
		}
        DWORD rs=ReadSwitches();
		writechar(UART_XBEE,',');
		writenib(UART_XBEE,rs>>12);
		writenib(UART_XBEE,rs>>8);
		writenib(UART_XBEE,rs>>4);
		writenib(UART_XBEE,rs);
		writechar(UART_XBEE,']');
        writechar(UART_XBEE,'\r');
        writechar(UART_XBEE,'\n');

	   if(Secs!=LastSec)
	   {
		iprintf("%d\r\n",(Seq-LastSeq));
		LastSec=Secs;
		LastSeq=Seq;
	   }
//       iprintf("T:%05X R:%05X E:%05X A:%05X  N:%05X %05X %05X %05X ",Throttle,Rudder,Elevator,Alieron,N[0],N[1],N[2],N[3]);
//       iprintf("%b\r\n",ReadSwitches());
	   LC=tc;
    }

	while(1)
		OSTimeDly(20);

}
