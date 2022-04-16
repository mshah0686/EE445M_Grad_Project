#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/LaunchPad.h"
#include "../inc/PLL.h"
#include "../inc/LPF.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/ADC.h"
#include "../inc/ADCT0ATrigger.h"
#include "../inc/IRDistance.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/Interpreter.h"
#include "../RTOS_Labs_common/ST7735.h"

void PortD_Init2(void){ 
  SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
  while((SYSCTL_RCGCGPIO_R&0x08)==0){};      
  GPIO_PORTD_DIR_R |= 0x0F;        // make PD3-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x0F;     // disable alt funct on PD3-0
  GPIO_PORTD_DEN_R |= 0x0F;        // enable digital I/O on PD3-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x0F;;    // disable analog functionality on PD
}


//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);

extern uint32_t NumCreated;   // number of foreground threads created
uint32_t IdleCount;    // CPU idle counter
uint32_t PIDWork;      // current number of PID calculations finished
uint32_t FilterWork;   // number of digital filter calculations finished
uint32_t NumSamples;   // incremented every ADC sample, in Producer
#define FS 400              // producer/consumer sampling
#define RUNLENGTH (20*FS)   // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 

#define PERIOD1 TIME_500US   // DAS 2kHz sampling period in system time units
#define PERIOD2 TIME_1MS     // PID period in system time units
int32_t x[64],y[64];           // input and output arrays for FFT

// Idle reference count for 10ms of completely idle CPU
// This should really be calibrated in a 10ms delay loop in OS_Init()
uint32_t IdleCountRef = 30769;
uint32_t CPUUtil;       // calculated CPU utilization (in 0.01%)

//---------------------User debugging-----------------------
uint32_t DataLost;     // data sent by Producer, but not received by Consumer
extern int32_t MaxJitter;             // largest time jitter between interrupts in usec
extern uint32_t const JitterSize;

extern int32_t MaxJitter4A;
extern int32_t MaxJitter2A;
extern uint32_t JitterHistogram4A[];
extern uint32_t JitterHistogram2A[];

//------------------Task 1--------------------------------
// 2 kHz sampling ADC channel 0, using software start trigger
// background thread executed at 2 kHz
// 60-Hz notch high-Q, IIR filter, assuming fs=2000 Hz
// y(n) = (256x(n) -503x(n-1) + 256x(n-2) + 498y(n-1)-251y(n-2))/256 (2k sampling)

//******** DAS *************** 
// background thread, calculates 60Hz notch filter
// runs 2000 times/sec
// samples analog channel 0, PE3,
// inputs:  none
// outputs: none
uint32_t DASoutput;
void DAS(void){ 
  uint32_t input;  
  if(NumSamples < RUNLENGTH){   // finite time run
    // PD0 ^= 0x01;
    input = ADC_In();           // channel set when calling ADC_Init
    // PD0 ^= 0x01;
    DASoutput = Filter(input);
    FilterWork++;        // calculation finished
    // PD0 ^= 0x01;
  }
}

//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die

// ***********ButtonWork*************
void ButtonWork(void){
  uint32_t myId = OS_Id(); 
  // PD1 ^= 0x02;
  ST7735_Message(1,0,"NumCreated   =",NumCreated); 
  // PD1 ^= 0x02;
  OS_Sleep(50);     // set this to sleep for 50msec
  ST7735_Message(1,1,"CPUUtil 0.01%=",CPUUtil);
  ST7735_Message(1,2,"DataLost     =",DataLost);
  ST7735_Message(1,3,"Jitter 0.1us =",MaxJitter);
  ST7735_Message(1,4,"CPUUtil 0.01%=",CPUUtil);
  // PD1 ^= 0x02;
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,128,2)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

//************SW2Push*************
// Called when SW2 Button pushed, Lab 3 only
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  if(OS_MsTime() > 20){ // debounce
    if(OS_AddThread(&ButtonWork,128,2)){
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
  }
}

//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------
// hardware timer-triggered ADC sampling at 400Hz
// Producer runs as part of ADC ISR
// Producer uses fifo to transmit 400 samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 2.5ms*64 = 160 ms (6.25 Hz), consumer sends data to Display via mailbox
// Display thread updates LCD with measurement

//******** Producer *************** 
// The Producer in this lab will be called from an ADC ISR
// A timer runs at 400Hz, started through the provided ADCT0ATrigger.c driver
// The timer triggers the ADC, creating the 400Hz sampling
// The ADC ISR runs when ADC data is ready
// The ADC ISR calls this function with a 12-bit sample 
// sends data to the consumer, runs periodically at 400Hz
// inputs:  none
// outputs: none
void Producer(uint32_t data){  
  if(NumSamples < RUNLENGTH){   // finite time run
    NumSamples++;               // number of samples
    if(OS_Fifo_Put(data) == 0){ // send to consumer
      DataLost++;
    } 
  } 
}

//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
void Display(void); 
void Consumer(void){ 
  uint32_t data,DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
  uint32_t t;                  // time in 2.5 ms
  ADC0_InitTimer0ATriggerSeq0(1, FS, &Producer); // start ADC sampling, channel 1, PE2, 400 Hz
  NumCreated += OS_AddThread(&Display,128,1); 
  while(NumSamples < RUNLENGTH) { 
    // PD2 = 0x04;
    for(t = 0; t < 64; t++){   // collect 64 ADC samples
      data = OS_Fifo_Get();    // get from producer
      x[t] = data;             // real part is 0 to 4095, imaginary part is 0
    }
    // PD2 = 0x00;
    cr4_fft_64_stm32(y,x,64);  // complex FFT of last 64 ADC values
    DCcomponent = y[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
    OS_MailBox_Send(DCcomponent); // called every 2.5ms*64 = 160ms
  }
  OS_Kill();  // done
}

//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
  uint32_t data,voltage,distance;
  uint32_t myId = OS_Id();
  ST7735_Message(0,1,"Run length = ",(RUNLENGTH)/FS); // top half used for Display
  while(NumSamples < RUNLENGTH) { 
    data = OS_MailBox_Recv();
    voltage = 3000*data/4095;   // calibrate your device so voltage is in mV
    distance = IRDistance_Convert(data,1); // you will calibrate this in Lab 6
    // PD3 = 0x08;
    ST7735_Message(0,2,"v(mV) =",voltage);  
    ST7735_Message(0,3,"d(mm) =",distance);  
    // PD3 = 0x00;
  } 
  ST7735_Message(0,4,"Num samples =",NumSamples);  
  OS_Kill();  // done
} 

//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------
// background thread that executes a digital controller 

//******** PID *************** 
// background thread, runs a PID controller
// runs every 1ms
// inputs:  none
// outputs: none
short IntTerm;     // accumulated error, RPM-sec
short PrevError;   // previous error, RPM
short Coeff[3] = { // PID coefficients
  384,  // 1.5 = 384/256 proportional coefficient
  128,  // 0.5 = 128/256 integral coefficient
  64    // 0.25 = 64/256 derivative coefficient*
};    
short Actuator;
void PID(void){ 
  static short err = -1000;  // speed error, range -100 to 100 RPM
  Actuator = PID_stm32(err,Coeff)/256;
  err++; 
  if(err > 1000) err = -1000; // made-up data
  PIDWork++;
}

//--------------end of Task 4-----------------------------

//------------------Task 5--------------------------------
// UART background ISR performs serial input/output
// Two software fifos are used to pass I/O data to foreground
// The interpreter runs as a foreground thread
// The UART driver should call OS_Wait(&RxDataAvailable) when foreground tries to receive
// The UART ISR should call OS_Signal(&RxDataAvailable) when it receives data from Rx
// Similarly, the transmit channel waits on a semaphore in the foreground
// and the UART ISR signals this semaphore (TxRoomLeft) when getting data from fifo

//******** Interpreter *************** 
// Modify your intepreter from Lab 1, adding commands to help debug 
// Interpreter is a foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
void Interpreter(void);    // just a prototype, link to your interpreter
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumSamples, NumCreated, MaxJitter, DataLost, FilterWork, PIDwork
      
// 2) print debugging parameters 
//    i.e., x[], y[] 

//--------------end of Task 5-----------------------------

//------------------Task 6--------------------------------
// foreground idle thread that always runs without waiting or sleeping

//******** Idle Task *************** 
// foreground thread, runs when nothing else does
// never blocks, never sleeps, never dies
// measures CPU idle time, i.e. CPU utilization
// inputs:  none
// outputs: none
void Idle(void){
  // measure idle time only for the first 20s for this lab	
  while(NumSamples < RUNLENGTH){
    IdleCount++;  // measure of CPU idle time
  }
  
  // compute CPU utilization (in 0.01%)
  CPUUtil = 10000 - (5*IdleCount)/IdleCountRef;
  
  while(1) {
    // if you do not wish to measure CPU utilization using this idle task
    // you can execute WaitForInterrupt to put processor to sleep
    WaitForInterrupt();
  }
}

//--------------end of Task 6-----------------------------

//*******************final user main DEMONTRATE THIS TO TA**********
int realmain(void){ // realmain
  OS_Init();        // initialize, disable interrupts
  PortD_Init2();     // debugging profile
  MaxJitter = 0;    // in 1us units
  DataLost = 0;     // lost data between producer and consumer
  IdleCount = 0;
  CPUUtil = 0;
  NumSamples = 0;
  FilterWork = 0;
  PIDWork = 0;
	
  // initialize communication channels
  OS_MailBox_Init();
  OS_Fifo_Init(64);    // ***note*** 4 is not big enough*****

  // hardware init
  ADC_Init(0);  // sequencer 3, channel 0, PE3, sampling in DAS() 

  // attach background tasks
  OS_AddSW1Task(&SW1Push,2);
  OS_AddSW2Task(&SW2Push,2);  // added in Lab 3
  OS_AddPeriodicThread(&DAS,PERIOD1,1); // 2 kHz real time sampling of PE3
  OS_AddPeriodicThread(&PID,PERIOD2,2); // Lab 3 PID, lowest priority

  // create initial foreground threads
  NumCreated = 0;
  NumCreated += OS_AddThread(&Consumer,128,1); 
  //NumCreated += OS_AddThread(&Interpreter,128,1); 
  NumCreated += OS_AddThread(&Idle,128,5);  // Lab 3, at lowest priority 
 
  OS_Launch(1*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}