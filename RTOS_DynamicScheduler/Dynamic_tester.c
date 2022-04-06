// Lab3.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Labs 2 and 3
// Lab2 Part 1: Testmain1 and Testmain2
// Lab2 Part 2: Testmain3, Testmain4, Testmain5, TestmainCS and realmain
// Lab3: Testmain6, Testmain7, TestmainCS and realmain (with SW2)

// Jonathan W. Valvano 1/29/20, valvano@mail.utexas.edu
// EE445M/EE380L.12
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// LED outputs to logic analyzer for use by OS profile 
// PF1 is preemptive thread switch
// PF2 is first periodic task (DAS samples PE3)
// PF3 is second periodic task (PID)
// PC4 is PF4 button touch (SW1 task)

// IR distance sensors
// J5/A3/PE3 analog channel 0  <- connect an IR distance sensor to J5 to get a realistic analog signal on PE3
// J6/A2/PE2 analog channel 1  <- connect an IR distance sensor to J6 to get a realistic analog signal on PE2
// J7/A1/PE1 analog channel 2
// J8/A0/PE0 analog channel 3  

// Button inputs
// PF0 is SW2 task (Lab3)
// PF4 is SW1 button input

// Analog inputs
// PE3 Ain0 sampled at 2kHz, sequencer 3, by DAS, using software start in ISR
// PE2 Ain1 sampled at 250Hz, sequencer 0, by Producer, timer tigger

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


//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);

uint32_t NumCreated;   // number of foreground threads created
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

#define PD0  (*((volatile uint32_t *)0x40007004))
#define PD1  (*((volatile uint32_t *)0x40007008))
#define PD2  (*((volatile uint32_t *)0x40007010))
#define PD3  (*((volatile uint32_t *)0x40007020))

void PortD_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
  while((SYSCTL_RCGCGPIO_R&0x08)==0){};      
  GPIO_PORTD_DIR_R |= 0x0F;        // make PD3-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x0F;     // disable alt funct on PD3-0
  GPIO_PORTD_DEN_R |= 0x0F;        // enable digital I/O on PD3-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x0F;;    // disable analog functionality on PD
}


//******** Interpreter *************** 
// Modify your intepreter from Lab 1, adding commands to help debug 
// Interpreter is a foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none
void Interpreter(void);    // just a prototype, link to your interpreter



/* BASIC ROUND ROBIN TEST */
// Two threads of same priority, should toggle PD1 and PD2 and switch between the two
void PD1Toggle_thread() {
  PD1 ^=0x02;
}

void PD2Toggle_thread() {
  PD2 ^= 0x04;
}

void PD3Toggle_thread() {
  PD3 ^= 0x08;
}

int Test1_Basic_RR_Test() {
  OS_Init();
  NumCreated = 0;
  PortD_Init();
  NumCreated += OS_AddThread(&PD1Toggle_thread, 128, 4);
  NumCreated += OS_AddThread(&PD2Toggle_thread, 128, 4);

  OS_Launch(TIME_2MS);
  return 0;
}


/***************** Two Threads Test *******************/
// Two threads running, one at priority 0, one at priority 1
// The second thread should update every 10 ticks and then fall down
// Checks for solution to starvation -> check PD2 toggles after some time 

int Test2_TwoThreads_OneAge() {
  OS_Init();
  NumCreated = 0;
  PortD_Init();
  NumCreated += OS_AddThread(&PD1Toggle_thread, 128, 0);
  NumCreated += OS_AddThread(&PD2Toggle_thread, 128, 1);

  OS_Launch(TIME_2MS);
  return 0;
}


/***************** Three Threads Test *******************/
// Three threads running, one at priority 0, one at priority 1, one at priority 1
// The second thread should update every 10 ticks and then fall down
// Checks for solution to starvation -> check PD2 toggles after some time 
int Test3_ThreeThreads_TwoAge() {
  OS_Init();
  NumCreated = 0;
  PortD_Init();
  NumCreated += OS_AddThread(&PD1Toggle_thread, 128, 0);
  NumCreated += OS_AddThread(&PD2Toggle_thread, 128, 1);
  NumCreated += OS_AddThread(&PD3Toggle_thread, 128, 1);
  OS_Launch(TIME_2MS);
  return 0;
}


/***************** Three Threads Test B *******************/
// Three threads running, one at priority 0, one at priority 1, one at priority 2
// Checks for solution to starvation -> check PD2 toggles after some time 
int Test3_ThreeThreads_TwoAge_DiffPri() {
  OS_Init();
  NumCreated = 0;
  PortD_Init();
  NumCreated += OS_AddThread(&PD1Toggle_thread, 128, 0);
  NumCreated += OS_AddThread(&PD2Toggle_thread, 128, 1);
  NumCreated += OS_AddThread(&PD3Toggle_thread, 128, 2);
  OS_Launch(TIME_2MS);
  return 0;
}

//*******************Trampoline for selecting main to execute**********
int main(void) { 			// main 
  return 0;
}
