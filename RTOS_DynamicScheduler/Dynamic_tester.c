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



uint32_t NumCreated;   // number of foreground threads created
uint32_t IdleCount;    // CPU idle counter
uint32_t PIDWork;      // current number of PID calculations finished
uint32_t FilterWork;   // number of digital filter calculations finished
uint32_t NumSamples;   // incremented every ADC sample, in Producer
#define FS 400              // producer/consumer sampling
#define RUNLENGTH (20*FS)   // display results and quit when NumSamples==RUNLENGTH
// 20-sec finite time experiment duration 


//---------------------User debugging-----------------------

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

uint32_t count1;
uint32_t count2;
uint32_t count3;
uint32_t error;

/* BASIC ROUND ROBIN TEST */
// Two threads of same priority, should toggle PD1 and PD2 and switch between the two
void PD0Toggle_thread() {
	while(1) {
		PD0 ^=0x01;
		count1++;
	}
}

void PD1Toggle_thread() {
	while(1) {
		PD1 ^=0x02;
		count1++;
	}
}

void PD2Toggle_thread() {
	while(1) {
		PD2 ^= 0x04;
		count2++;
	}
}

void PD3Toggle_thread() {
	while(1) {
		PD3 ^= 0x08;
		count3++;
	}
}

void PF0Toggle_thread() {
	while(1) {
		PF0 ^= 0x01;
		count1++;
	}
}

void PF1Toggle_thread() {
	while(1) {
		PF1 ^=0x02;
		count1++;
	}
}

void PF2Toggle_thread() {
	while(1) {
		PF2 ^= 0x04;
		count2++;
	}
}

void PF3Toggle_thread() {
	while(1) {
		PF3 ^= 0x08;
		count3++;
	}
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
	count1 = 0;
	count2 = 0;
	count3 = 0;
	error = 0;
  PortD_Init();
  NumCreated += OS_AddThread(&PF1Toggle_thread, 128, 0);
  NumCreated += OS_AddThread(&PF2Toggle_thread, 128, 1);

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
  OS_Launch(TIME_2MS*5);
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
  NumCreated += OS_AddThread(&PD3Toggle_thread, 128, 5);
  OS_Launch(TIME_2MS);
  return 0;
}


int Test4_ManyThreads_DiffPri() {
  OS_Init();
  NumCreated = 0;
  PortD_Init();

  NumCreated += OS_AddThread(&PD0Toggle_thread, 128, 0);
  NumCreated += OS_AddThread(&PD1Toggle_thread, 128, 1);
  NumCreated += OS_AddThread(&PD2Toggle_thread, 128, 2);
  NumCreated += OS_AddThread(&PD3Toggle_thread, 128, 3);
  //NumCreated += OS_AddThread(&PF0Toggle_thread, 128, 0);
  NumCreated += OS_AddThread(&PF1Toggle_thread, 128, 4);
  NumCreated += OS_AddThread(&PF2Toggle_thread, 128, 5);
  //NumCreated += OS_AddThread(&PF3Toggle_thread, 128, 4);

  OS_Launch(TIME_2MS);
  return 0;
}


/***************** Sleep Tests 1: 2 Threads *******************/
// Two threads, higher priority sleeps after toggling
// Lower prority runs for some time until higher priority wakes

void PF1Toggle_sleepy_thread() {
	while(1) {
		PF1 ^=0x02;
		count1++;
    PF1 ^= 0x02;
    OS_Sleep(10);
	}
}

int Sleep_Test_2Thread_One_Sleep() {
  OS_Init();
  NumCreated = 0;
  PortD_Init();
  NumCreated += OS_AddThread(&PF1Toggle_sleepy_thread, 128, 0);
  NumCreated += OS_AddThread(&PD2Toggle_thread, 128, 1);
  OS_Launch(TIME_2MS);
  return 0;
}

/***************** Sleep Tests 2: 3 Threads *******************/
void PF2Toggle_sleepy_thread() {
  while(1) {
    PF2 ^= 0x04;
    OS_Sleep(10);
  }
}

// PF3 thread should never age -> should run when other two threads asleep
int Sleep_Test2_3Thread_Two_Sleep() {
  OS_Init();
  NumCreated = 0;
  PortD_Init();
  NumCreated += OS_AddThread(&PF1Toggle_sleepy_thread, 128, 0);
  NumCreated += OS_AddThread(&PF2Toggle_sleepy_thread, 128, 1);
  NumCreated += OS_AddThread(&PF3Toggle_thread, 128, 2);
  OS_Launch(TIME_2MS);
  return 0;
}


/******************** Kill Test: 1 High dies **********/
void PF2_Suicide_Thread() {
	PF2 ^= 0x04;
	PF2 ^= 0x04;
	OS_Kill();
}

void PF0_Suicide_Sponser() {
	while(1) {
		PF1 ^= 0x02;
		OS_AddThread(&PF2_Suicide_Thread, 128, 0);
	}
	
}

int Kill_Test_1Dies() {
  OS_Init();
  NumCreated = 0;
  NumCreated += OS_AddThread(&PF0_Suicide_Sponser, 128, 1);
  OS_Launch(TIME_2MS);
  return 0;
}


/******************** Semaphore Test: 1 Simple **********/
Sema4Type simple_sema;

void sema_thread1() {
	OS_Wait(&simple_sema);
	PF2 = 0x04;
	OS_Signal(&simple_sema);
	OS_Kill();
}

void sema_thread2() {
	OS_Wait(&simple_sema);
	PF1 = 0x02;
	OS_Signal(&simple_sema);
	OS_Kill();
}

void sema_thread3() {
	OS_Wait(&simple_sema);
	PF3 = 0x08;
	OS_Signal(&simple_sema);
	OS_Kill();
}

void idle() {
	while(1);
}

void sema_test1_all_capture() {
	OS_Init();
	OS_InitSemaphore(&simple_sema, 1);
  NumCreated = 0;
  NumCreated += OS_AddThread(&sema_thread3, 128, 1);
  NumCreated += OS_AddThread(&sema_thread2, 128, 1);
	NumCreated += OS_AddThread(&sema_thread1, 128, 1);
	NumCreated += OS_AddThread(&idle, 128, 4);
	OS_Launch(TIME_2MS);
}


/******* SEMA TEST PMP ************/
// One thread started at low priority
// Thread captures semaphore
// Creates a higher level thread
// Higher level blocked on the semaphore
// Lower lever should be aged to match the higher level
// Then run till released and deaged, 
Sema4Type pmp_test_sema;

void higher_stalled_thread() {
  PF3 ^= 0x08;
  OS_Wait(&pmp_test_sema);    // SHOULD BLOCK and raise other thread
  PF3 ^= 0x08;
  OS_Signal(&pmp_test_sema);
  OS_Kill();
}

void lower_blocking_thread() {
  OS_Wait(&pmp_test_sema);
  PF2 ^= 0x04;
  OS_AddThread(&higher_stalled_thread, 128, 0);
  PF2 ^= 0x04;
  OS_Signal(&pmp_test_sema);
  PF2 ^= 0x04;
  OS_Kill();
}

void sema_test2_all_capture() {
	OS_Init();
	OS_InitSemaphore(&pmp_test_sema, 1);
  NumCreated = 0;
  NumCreated += OS_AddThread(&lower_blocking_thread, 128, 4);
	OS_Launch(TIME_2MS);
}

/******* SEMA TEST PMP Multiple Threads ************/
// High priority threads run
// Low priroity grabs semaphore, spawns middle thread
// Low priroity should be aged to match middle, then also keep again until it reaches high thread
// Should not deage until semaphore released 

// TODO: Low priority ages to high then should fall back to middle in pre-emption

void high_thread_toggling() {
  /* Just toggles LED at high priority */
  while(1) {
      PF1 ^= 0x02;
  }
}

void middle_thread_blocked() {
  /* Blocked on semaphore from lower thread */
  PF3 ^= 0x08;
  OS_Wait(&pmp_test_sema);    // SHOULD BLOCK and raise other thread
  PF3 ^= 0x08;
  OS_Signal(&pmp_test_sema);
  OS_Kill();
}

void lower_thread_with_semaphore() {
  /* Should work really slowly, then get aged to middle thread priority */
  while(1) {
    OS_Wait(&pmp_test_sema);
    PF2 ^= 0x04;
    OS_AddThread(&higher_stalled_thread, 128, 3);
    PF2 ^= 0x04;
    OS_Signal(&pmp_test_sema);
    PF2 ^= 0x04;
  }
  /* Should not deage until OS_Signal */
}

void sema_test3_all_capture() {
	OS_Init();
	OS_InitSemaphore(&pmp_test_sema, 1);
  NumCreated = 0;
  NumCreated += OS_AddThread(&high_thread_toggling, 128, 0);
  NumCreated += OS_AddThread(&lower_thread_with_semaphore, 128, 4);
	OS_Launch(TIME_2MS);
}

//*******************Trampoline for selecting main to execute**********
int main(void) { 			// main 
  sema_test2_all_capture();
}
