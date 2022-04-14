// *************os.c**************
// EE445M/EE380L.6 Labs 1, 2, 3, and 4 
// High-level OS functions
// Students will implement these functions as part of Lab
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 
// Jan 12, 2020, valvano@mail.utexas.edu


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/CortexM.h"
#include "../inc/PLL.h"
#include "../inc/LaunchPad.h"
#include "../inc/Timer4A.h"
#include "../inc/Timer5A.h"
#include "../inc/WTimer0A.h"
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../inc/TCB.h"
#include "../inc/Timer3A.h"
#include "../inc/Timer2A.h"



/**
  * List of TODOs
  *
  *
  *
  */

// Performance Measurements 
int32_t MaxJitter;             // largest time jitter between interrupts in usec
uint32_t const JitterSize=JITTERSIZE;
uint32_t JitterHistogram[JITTERSIZE]={0,};

int32_t MaxJitter4A;
int32_t MaxJitter2A;
uint32_t JitterHistogram4A[JITTERSIZE]={0,}; /* Collects jitter on Timer4A */
uint32_t JitterHistogram2A[JITTERSIZE]={0,}; /* Collects jitter on Timer2A */

uint32_t NumThreadsCreated;

uint32_t SystemMSCounter;
uint32_t MalleableMSCounter;

/* For sleep and signal -> waking thread of higher priority*/
uint8_t higher_pri_thread_added_flag;

/* Measuring interrupts disabled time */
uint32_t int_max_disable_time;
long int_total_disable_time;
uint32_t int_last_disable_time;

void set_int_disable_time () {
	int_last_disable_time = OS_Time();
}

void get_int_disable_time () {
	uint32_t current = OS_Time();
	/* If timer wraped over when interrupts disabled */
	if(current < int_last_disable_time) {
		current += 80000;
	}
	uint32_t diff = OS_TimeDifference(int_last_disable_time, current);
	int_total_disable_time += diff;
	if(diff > int_max_disable_time) {
		int_max_disable_time = diff;
	}
}

long get_int_percentage_time () {
	return (int_total_disable_time) / (OS_Time() / 10000000);
}

// Defines for MS Timer
#define TIMER5_MS_TIME_PRI 2
#define TIMER5_MS_TIME_PERIOD 80000

/* TCB Circular LL struct */
struct tcb* RunningPtrs[6];     /* pointers to list of running threads for each priority level */
struct tcb* current_thread_ptr;

/* LL struct for sleeping threads */
struct tcb* sleep_list_head;

void StartOS();             /* defined in osasm.s */
void ContextSwitch();       /* defined in osasm.s */
void DisableInterrupts();   /* defined in startup.s */
void EnableInterrupts();    /* defined in startup.s */

void GPIOPortF_Init(void);

void OS_CountMsTime_Timer5_UserTask(); /*Task for Timer 5 /MS Timer */
void OS_Time_Init();
void OS_Increment_Time();

/* Function to add tcb to global running linked list */
void add_running_thread(struct tcb* to_add) {
  uint32_t priority = to_add->aged_priority;
  
  if (current_thread_ptr == NULL)         /* if there is no currently running thread, run this one */
    current_thread_ptr = to_add;
  
	if(RunningPtrs[priority] == NULL) {     /* set up the thread in the correct priority list */
		RunningPtrs[priority] = to_add;
		to_add->next = to_add;
		to_add->previous = to_add;
	} else {
		to_add->previous = RunningPtrs[priority]->previous;
		to_add->next = RunningPtrs[priority];
		
		RunningPtrs[priority]->previous->next = to_add;
		RunningPtrs[priority]->previous = to_add;
	}
}

/* Function to remove TCB from global running linked list 
  Updats runningPtrs to correct pointer*/
void remove_running_thread(struct tcb* to_remove) {
  uint32_t priority = to_remove->aged_priority;
  /* if the to_remove is head */
  if (RunningPtrs[priority] == to_remove) {
    /* If only thing in list */
    if (RunningPtrs[priority]->next == to_remove) {
      RunningPtrs[priority] = NULL;
    }
    /* Move RunningPtrs over */
    else {
      RunningPtrs[priority] = to_remove->next;
    }
  }
  
  /* update next and previous poitners */
	to_remove->previous->next = to_remove->next;
	to_remove->next->previous = to_remove->previous;
}

void check_for_falling_priority() {
  for (int i = 0; i < 6; i++) {
    if (RunningPtrs[i] != NULL) {
      current_thread_ptr->next = RunningPtrs[i];
      return;
    }
  }
}

void check_for_rising_priority() {
  uint32_t curr_priority = current_thread_ptr->aged_priority;
  for (int i = 0; i < 6; i++) {
    if (RunningPtrs[i] != NULL && i < curr_priority) {
      RunningPtrs[curr_priority] = current_thread_ptr->next;
      current_thread_ptr->jumpFlag = 1;
      current_thread_ptr->jumpTo = RunningPtrs[i];
      OS_Suspend();
      return;
    }
  }
}

/* Takes a thread and moves it in the RunPtrs based on aged_priority */
/* Will add at tail of list and update RunPtrs as necessary*/
void update_thread_priority(tcb* thread) {
  remove_running_thread(thread);
  add_running_thread(thread);
}

/* Goes through running ptrs and finds next thread to run in highes priority */
tcb* find_next_best_thread_run(void) {
  for(int i = 0; i < TOTAL_PRIORITIES; i++) {
    if(RunningPtrs[i] != NULL) {
      return RunningPtrs[i];
    }
  } return NULL;
}

/*------------------------------------------------------------------------------
  Systick Interrupt Handler
  SysTick interrupt happens every 10 ms
  used for preemptive thread switch
 *------------------------------------------------------------------------------*/
void SysTick_Handler(void) {
	DisableInterrupts();
  /* Current_thread_pointer->ticks = 0 : reset current thread counter bc it just ran */
  current_thread_ptr->ticks = 0;

  /* Update ticks and age certain threads */
  /* for heads in run pointer with priority lower than current */
  for(int i = current_thread_ptr->aged_priority +1; i < 6; i++) {
    if(RunningPtrs[i] != NULL) {
			/* run ptrs-> head.ticks ++ */
			RunningPtrs[i]->ticks++;
			/* If ticks are at 10 -> need to age */
			if(RunningPtrs[i]->ticks == TICK_COUNT* RunningPtrs[i]->aged_priority) {
				/* ticks = 0 */
				RunningPtrs[i]->ticks = 0;
				
				/* Age thread and update in list */
				tcb* thread_to_age = RunningPtrs[i];
				remove_running_thread(thread_to_age);
				thread_to_age->aged_priority--;
				add_running_thread(thread_to_age);
			}
		}
  }

  /* De-age the current_thread if needed */
  if(current_thread_ptr-> aged_priority != current_thread_ptr->orig_priority 
      && current_thread_ptr->pmp_flag == 0) {
    /* current_thread -> aged_priroity = orig_priority*/
		tcb* thread_to_age = current_thread_ptr;
		remove_running_thread(thread_to_age);
		thread_to_age->aged_priority = thread_to_age->orig_priority;
		add_running_thread(thread_to_age);
		
    /* if priority update happened -> find next best thread to run */
    tcb* next_to_run = find_next_best_thread_run();
    /* Set next best_thread ticks to 0 */
    current_thread_ptr->jumpFlag = 1;
    /* Set jump flag and jump tcb */
    current_thread_ptr->jumpTo = next_to_run;

    /* Update run ptrs ??*/ //TODO
  } else if(higher_pri_thread_added_flag > 0) {
    /* Clear flag */
    higher_pri_thread_added_flag = 0;
		
    /* find next best to run */
    tcb* next_to_run = find_next_best_thread_run();
    current_thread_ptr->jumpFlag = 1;
    current_thread_ptr->jumpTo = next_to_run;
  } else {
    /* No deaging, just do simple round robin */
    RunningPtrs[current_thread_ptr->aged_priority] = current_thread_ptr->next;
  }
  /* If no de-aging, use round robin within that priority */
  ContextSwitch();		// Change threads
	EnableInterrupts();
}


void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;  // reload value
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it    
  NVIC_ST_CTRL_R = 0x07;			  // enable SysTick with core clock and interrupts
}

/**
 * @details  Initialize operating system, disable interrupts until OS_Launch.
 * Initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers.
 * Interrupts not yet enabled.
 * @param  none
 * @return none
 * @brief  Initialize OS
 */
void OS_Init(void){
	/* Start System MS time */
  OS_Time_Init();
	
  /* Enable interrupts by calling StartOS() */
	DisableInterrupts();
  
  /* Run at 80 MHz */
  PLL_Init(Bus80MHz);
  
	/* Set up TCB Structure */
	tcb_mem_init();
  
	/* Start Malleable MS time */
	Timer5A_Init(&OS_CountMsTime_Timer5_UserTask, TIMER5_MS_TIME_PERIOD, TIMER5_MS_TIME_PRI);
  OS_StartMsTime();
  
  /* Enable Interpreter */
  UART_Init();
  ST7735_InitR(INITR_REDTAB);
  
  /* Enable PortF switches */
  GPIOPortF_Init();
	
	/* Set ptrs for global linked lists */
  for (int i = 0; i < 6; i++) {
    RunningPtrs[i] = NULL;
  }
	sleep_list_head = NULL;
	NumThreadsCreated = 0;
}; 

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, int32_t value){
  semaPt->Value = value;
	semaPt->head = NULL;
}; 

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
  DisableInterrupts();
	semaPt->Value--;
  if(semaPt->Value < 0) {
		/* Remove TCB from current running linked list */
		remove_running_thread(current_thread_ptr);
		/* Add TCB to proper location in this list */
		if(semaPt->head ==NULL) {
			semaPt->head = current_thread_ptr;
      current_thread_ptr->sema_next = NULL;
		} else if (semaPt->head->aged_priority > current_thread_ptr->aged_priority){
      current_thread_ptr->sema_next = semaPt->head;
      semaPt->head = current_thread_ptr;
    }
    else {
      /* Add to end or add to where priority is lower than current priority */
      struct tcb* current = semaPt->head->sema_next;
      struct tcb* previous = semaPt->head;
      
      while(current && current->aged_priority <= current_thread_ptr->aged_priority) {
        previous = current;
        current = current->sema_next;
      }
      /* Add between current and previous */
      current_thread_ptr->sema_next = current;
      previous->sema_next = current_thread_ptr;
    }

		/* Check for any priority ceiling/inversion protocols */
		/* Check the aged priority of current holding thread */
		if(semaPt->current_holding_thread->aged_priority > current_thread_ptr->aged_priority) {
			/* Need to priority match the current holding thread */
			remove_running_thread(semaPt->current_holding_thread);
			
			/* Update priority */
			semaPt->current_holding_thread->aged_priority = current_thread_ptr->aged_priority;
			
			/* Add properly to the list */
			add_running_thread(semaPt->current_holding_thread);

      /* set flag so it does not get deaged in premption */
      semaPt->current_holding_thread->pmp_flag++;
		}
		
		/* Context switch to next best thread*/
    tcb* next = find_next_best_thread_run();
		current_thread_ptr->jumpFlag = 1;
		current_thread_ptr->jumpTo = next;
		
		OS_Suspend();
	} else {
		/* Update current_thread_ptr */
		semaPt->current_holding_thread = current_thread_ptr;
	}
  EnableInterrupts();
}; 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
  long sr = StartCritical();
	/* Increment value */
  semaPt->Value++;
  
	/* There is someone blocked on this pointer */
	if(semaPt->Value <= 0) {
		/* Add to current running list */
		add_running_thread(semaPt->head);
    
		/* Update current holding thread */
		semaPt->current_holding_thread = semaPt->head;
		
		/* Remove from semaphore list */
		semaPt->head = semaPt->head->sema_next;
		
    /* If current thread was pmp_flaged, then properly deage the thread */
    /* Set signal for PMP */
    if(current_thread_ptr->pmp_flag == 1) {
      /* set pmp_flag to 0 */
      current_thread_ptr->pmp_flag = 0;
      
      /* remove and then add thread to running list properly */
      remove_running_thread(current_thread_ptr);
      current_thread_ptr->aged_priority = current_thread_ptr->orig_priority;
      add_running_thread(current_thread_ptr);
      
    } else if (current_thread_ptr->pmp_flag > 1) {
      /* Set pmp flag properly */
      current_thread_ptr->pmp_flag--;
    }

		/* find next thread to run */
		tcb* next = find_next_best_thread_run();
		
		/* Context swtich if new thread of higher priority */
		if(current_thread_ptr->aged_priority > next->aged_priority) {
			current_thread_ptr->jumpFlag = 1;
			current_thread_ptr->jumpTo = next;
			OS_Suspend();
		}
	} else {
		semaPt->current_holding_thread = NULL;
	}
  EndCritical(sr);
}; 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
  DisableInterrupts();
	semaPt->Value--;
	if(semaPt->Value < 0) {
		/* Remove TCB from current running linked list */
		remove_running_thread(current_thread_ptr);
		/* Add TCB to proper location in this list */
		if(semaPt->head == NULL) {
			semaPt->head = current_thread_ptr;
      current_thread_ptr->sema_next = NULL;
		} else if (semaPt->head->aged_priority > current_thread_ptr->aged_priority){
      current_thread_ptr->sema_next = semaPt->head;
      semaPt->head = current_thread_ptr;
    } else {
			/* Add to end or add to where priority is lower than current priority */
			struct tcb* current = semaPt->head->sema_next;
			struct tcb* previous = semaPt->head;
			while(current && current->aged_priority >= current_thread_ptr->aged_priority) {
				previous = current;
				current = current->sema_next;
			}
			/* Add between current and previous */
			current_thread_ptr->sema_next = current;
			previous->sema_next = current_thread_ptr;
		}
		/* Context switch */
    check_for_falling_priority();
		OS_Suspend();
	}
  EnableInterrupts();
}; 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
  long sr = StartCritical();
	/* There is someone blocked on this pointer */
	if(semaPt->Value < 0) {
		/* Add to current running list */
		add_running_thread(semaPt->head);
    check_for_rising_priority();
		/* Remove from semaphore list */
		semaPt->head = semaPt->head->sema_next;
		semaPt->Value++;
	} else {
		semaPt->Value = 1;
	}
  EndCritical(sr);
}; 

void SetInitialStack(uint32_t* stack_ptr){
  *(--stack_ptr) = 0x01000000;   // thumb bit (dummy PSR)
  --stack_ptr;                   // PC
  *(--stack_ptr) = 0x14141414;   // R14
  *(--stack_ptr) = 0x12121212;   // R12
  *(--stack_ptr) = 0x03030303;   // R3
  *(--stack_ptr) = 0x02020202;   // R2
  *(--stack_ptr) = 0x01010101;   // R1
  *(--stack_ptr) = 0x00000000;   // R0
  *(--stack_ptr) = 0x11111111;   // R11
  *(--stack_ptr) = 0x10101010;   // R10
  *(--stack_ptr) = 0x09090909;   // R9
  *(--stack_ptr) = 0x08080808;   // R8
  *(--stack_ptr) = 0x07070707;   // R7
  *(--stack_ptr) = 0x06060606;   // R6
  *(--stack_ptr) = 0x05050505;   // R5
  *(--stack_ptr) = 0x04040404;   // R4
}

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), uint32_t stackSize, uint32_t priority){  
	/* Stack size checking */
	if(stackSize%8 != 0) return 0;
	if(priority > 5) return 0;
  
  /* Start atomic section to make OS_AddThread reentrant */
  long sr = StartCritical();
	
	struct tcb* new_tcb = tcb_mem_get_tcb();
	
	if(new_tcb == NULL) {
		/* out of memory */
    EndCritical(sr);
		return 0;
	}
	
	/* set up new thread */
	new_tcb->orig_priority = priority;
  new_tcb->aged_priority = priority;
  new_tcb->ticks = 0;

	/* adding thread to TCB linked list */
	add_running_thread(new_tcb);
	
  /* set up the initial stack */
  SetInitialStack(new_tcb->stack_pointer);
  *(new_tcb->stack_pointer - 2) = (int32_t)task;    /* PC */
  new_tcb->stack_pointer -=16;                      /* reset stack pointer to account for pushes */
	
	NumThreadsCreated++;
  
  /* End atomic section */
  tcb* next = find_next_best_thread_run();
	if(current_thread_ptr && next != current_thread_ptr) {
		current_thread_ptr->jumpFlag = 1;
		current_thread_ptr->jumpTo = next;
		OS_Suspend();
	}
  EndCritical(sr);
  
  return 1;
};

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
uint32_t OS_Id(void){
  // put Lab 2 (and beyond) solution here
  
  return 0; // replace this line with solution
};


//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In lab 1, this command will be called 1 time
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
uint8_t Timer4Used = 0;
uint8_t Timer2Used = 0;
int OS_AddPeriodicThread(void(*task)(void), uint32_t period, uint32_t priority){
  if (!Timer4Used) {
    Timer4A_Init(task, period, priority);
    Timer4Used = 1;
  }
  else if (!Timer2Used) {
    Timer2A_Init(task, period, priority);
    Timer2Used = 1;
  }
  else {
    return 0;
  }
  return 1;
};

void (*SW1_Task)(void);   // function to run on SW1 pressed
void (*SW2_Task)(void);

void GPIOPortF_Init(void) {
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  GPIO_PORTF_DIR_R |=  0x0E;    // output on PF3,2,1 
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF0,4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF0,4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF0,4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag 0,4
}

/*----------------------------------------------------------------------------
  PF1 Interrupt Handler
 *----------------------------------------------------------------------------*/
void GPIOPortF_Handler(void){
  if (GPIO_PORTF_RIS_R & 0x01) {
    GPIO_PORTF_ICR_R = 0x01;
    if (OS_MsTime() > 20) {
      SW2_Task();
    }
    OS_ClearMsTime();
  }
  else {
    /* Clear interrupt */
    GPIO_PORTF_ICR_R = 0x10;
    // PF1 ^= 0x02;
    if (OS_MsTime() > 20) {
      /* Run task */
      SW1_Task();
    }
    OS_ClearMsTime();
  }
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal   OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads

int OS_AddSW1Task(void(*task)(void), uint32_t priority){
  GPIO_PORTF_IM_R |= 0x10;                               // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|priority << 21; // (g) priority is BIT 21 in NVIC_PRI7_R
  NVIC_EN0_R = 0x40000000;                               // (h) enable interrupt 30 in NVIC
	/* store pointer to task */
	SW1_Task = task;
  return 1; // replace this line with solution
};

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), uint32_t priority){
  GPIO_PORTF_IM_R |= 0x01;                               // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|priority << 21; // (g) priority is BIT 21 in NVIC_PRI7_R
  NVIC_EN0_R = 0x40000000;                               // (h) enable interrupt 30 in NVIC
  
  SW2_Task = task;
  return 1;
};


// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
  /* Start atomic section to make OS_Sleep reentrant */
  DisableInterrupts();
  
  /* OS_Sleep(0) */
  if (sleepTime == 0) {
    OS_Suspend();
    EnableInterrupts();
    return;
  }
  
	/* Remove from linked list */
	remove_running_thread(current_thread_ptr);

  /* Reset aging and ticks because this thread is sleeping */
  current_thread_ptr->aged_priority = current_thread_ptr->orig_priority;
  current_thread_ptr->ticks = 0;
  
	/* Move TCB to seperate linked list */
	if(sleep_list_head == NULL) {
		/* add to head */
		current_thread_ptr->sleep_next = NULL;
		current_thread_ptr->sleep_count = sleepTime;
		sleep_list_head = current_thread_ptr;
	} else {
		if(current_thread_ptr == sleep_list_head) {
			return; 	// THIS SHOULD ENVER HAPPEN
		}
		/* Find place to insert in list */
		/* Calculate difference from head */
		int32_t difference = sleepTime - sleep_list_head->sleep_count;
		/* If difference < 0; insert at head */
		if(difference < 0) {
			current_thread_ptr->sleep_next = sleep_list_head;
      current_thread_ptr->sleep_count = sleepTime;
      sleep_list_head = current_thread_ptr;
		} else {
			/* If difference > 0, go through list and find position to add at */
			struct tcb* current = sleep_list_head->sleep_next;
			struct tcb* previous = sleep_list_head;
			/* while difference is greater than the current tcb, go to next tcb */
			while(current && difference > current->sleep_count) {
				previous = previous->sleep_next;
				current = current->sleep_next;
			}
			/* add at current (current could be null at the end of list */
			previous->sleep_next = current_thread_ptr;
			current_thread_ptr->sleep_next = current;
			current_thread_ptr->sleep_count = difference;
		}
	}

  TIMER5_TAV_R = TIMER5_MS_TIME_PERIOD - 1;
  
	/* Context switch to new thread: next best thread to run */
  tcb* next_thread = find_next_best_thread_run();
  current_thread_ptr->jumpFlag = 1;
  current_thread_ptr->jumpTo = next_thread;

	OS_Suspend();  // NOTE: thread going to sleep has to preserve next pointer
  EnableInterrupts();
};  

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
  /* Start atomic section to make OS_Kill reentrant */
  DisableInterrupts();
  
  /* Remove from linked list */
	remove_running_thread(current_thread_ptr);
	
	/* Free the tcb */
	tcb_mem_free_tcb(current_thread_ptr);
	
	/* Context switch to next thread */
  struct tcb* next = find_next_best_thread_run();
	current_thread_ptr->jumpFlag = 1;
	current_thread_ptr->jumpTo = next;
	
	current_thread_ptr->ticks = 0;

	OS_Suspend();
 
	/* End atomic section */
  EnableInterrupts();
}; 

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
  long sr = StartCritical();
  NVIC_ST_CURRENT_R = 0;
  ContextSwitch();
  EndCritical(sr);
}
  
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
# define FIFO_SIZE 32

Sema4Type FIFO_sema_room_avail;
Sema4Type FIFO_sema_data_avail;
uint8_t read_index;
uint8_t write_index;
uint32_t FIFO[FIFO_SIZE];

void OS_Fifo_Init(uint32_t size){ //TODO size with dynamic allocation
  // put Lab 2 (and beyond) solution here
  OS_InitSemaphore(&FIFO_sema_room_avail, FIFO_SIZE);
	OS_InitSemaphore(&FIFO_sema_data_avail, 0);
	read_index = 0;
	write_index = 0;
};

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(uint32_t data){
  // put Lab 2 (and beyond) solution here
	long sr = StartCritical();
	uint8_t ret = 0;
	if(OS_Fifo_Size() < FIFO_SIZE) {
		/* There is space */
		FIFO[write_index] = data;
		write_index = (write_index+1) % FIFO_SIZE;
		OS_Signal(&FIFO_sema_data_avail);
		ret = 1;
	}
	EndCritical(sr);
  return ret; // replace this line with solution
};

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
uint32_t OS_Fifo_Get(void){
  // put Lab 2 (and beyond) solution here
	OS_Wait(&FIFO_sema_data_avail);
	uint32_t data = FIFO[read_index];
	read_index = (read_index+1) % FIFO_SIZE;
  //OS_Signal(&FIFO_sema_room_avail);
  return data; // replace this line with solution
};

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
int32_t OS_Fifo_Size(void){
  // put Lab 2 (and beyond) solution here
  //TODO Do we need mutex when reading read and write pointers?
	uint8_t read_cp = read_index;
	uint8_t write_cp = write_index;
	if(read_cp > write_cp) {
		write_cp += FIFO_SIZE;
	} 
	return write_cp - read_cp;
};

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
Sema4Type MailBox_Free;
Sema4Type MailBox_DataAvailable;
uint32_t MailBoxData;
void OS_MailBox_Init(void){
  OS_InitSemaphore(&MailBox_Free, 1);
  OS_InitSemaphore(&MailBox_DataAvailable, 0);
};

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(uint32_t data){
  OS_bWait(&MailBox_Free);
  MailBoxData = data;
  OS_bSignal(&MailBox_DataAvailable);
};

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
uint32_t OS_MailBox_Recv(void){
  OS_bWait(&MailBox_DataAvailable);
  uint32_t data = MailBoxData;
  OS_bSignal(&MailBox_Free);
  return data;
};

// ******** OS_Time_Init *******
void OS_Time_Init(void) {
  Timer3A_Init(OS_Increment_Time, 80000, 2);
}

// ******** OS_Increment_Time **
void OS_Increment_Time() {
  SystemMSCounter++;
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
uint32_t OS_Time(void){
  uint32_t time = (SystemMSCounter * 80000) + (79999 - TIMER3_TAR_R);
  return time;
};

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// 32-bit precision, 1us resolution
uint32_t OS_TimeDifference(uint32_t start, uint32_t stop){
  return stop - start;
};

/* User count timer flag */
uint8_t MalleableMsFlag;

//********* OS_CountMsTime_Timer5_UserTask**************
// Increases the count by 1 every interrupt on Timer5A to track OS timing
void OS_CountMsTime_Timer5_UserTask() {
	DisableInterrupts();
  
	if(MalleableMsFlag) MalleableMSCounter = MalleableMSCounter + 1;
	
	/* Check sleeping thread at head to wake if done */
	if(sleep_list_head) sleep_list_head->sleep_count--;
	struct tcb* temp = NULL;
	
	/* If head is done -> add to global list*/
	while(sleep_list_head && sleep_list_head->sleep_count ==0) {
    /* Reset ticks and priorities */
    sleep_list_head->aged_priority = sleep_list_head->orig_priority;
    sleep_list_head->ticks = 0;

		/* Add tcb to running linked list */
		add_running_thread(sleep_list_head);
		
		/* Move sleep head */
		temp = sleep_list_head;
		sleep_list_head = sleep_list_head->sleep_next;
		temp->sleep_next = NULL;
	}

  /* Set special flag for next systick handler to check for higher priority thread */
  if(temp && temp->orig_priority < current_thread_ptr->aged_priority) {
    higher_pri_thread_added_flag = 1;
  }
	EnableInterrupts();
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (solve for Lab 1), keeps timer going
// Inputs:  none
// Outputs: none
/* Using Timer 5 interrupts to create a 1ms timer -> then return ms time*/
void OS_ClearMsTime(void) {
  // put Lab 1 solution here
  //TIMER5_TAV_R = 0x00;
	MalleableMSCounter = 0;
	//TIMER5_TAV_R = TIMER5_MS_TIME_PERIOD -1;
}

// ******** OS_StopMsTime ************
// Stops the Ms timer
// Inputs:  none
// Outputs: none
/* Using Timer 5 interrupts to create a 1ms timer -> then return ms time*/
void OS_StopMsTime(void) {
	//Timer5A_Stop();
	MalleableMsFlag = 0;
}

// ******** OS_StartMsTime ************
// Start a periodic interrupt, does not clear the original ms value
// Inputs:  none
// Outputs: none
/* Using Timer 5 interrupts to create a 1ms timer -> then return ms time*/
void OS_StartMsTime(void){
	//Timer5A_Init(&OS_CountMsTime_Timer5_UserTask, TIMER5_MS_TIME_PERIOD, TIMER5_MS_TIME_PRI);
	MalleableMsFlag = 1;
}

// ******** OS_MsTime ************
// reads the current time in msec (solve for Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// NOTE: Last digit is fixed point, with Delta = 10: divide by 10 for MS
// For Labs 2 and beyond, it is ok to make the resolution to match the first call to OS_AddPeriodicThread
uint32_t OS_MsTime(void){
	/* FOR FINE GRAIN TIMER, UNCOMMENT LINES -> NOTE: Messes with threads sleeping */
	//uint32_t current_val = (TIMER5_TAR_R % TIMER5_MS_TIME_PERIOD) * 10;
	//current_val = 10 - (current_val/(TIMER5_MS_TIME_PERIOD-1));			
  return MalleableMSCounter;
}


//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(uint32_t theTimeSlice){
	SysTick_Init(theTimeSlice);  // For preemptive scheduling
  StartOS();
};

//************** I/O Redirection *************** 
// redirect terminal I/O to UART or file (Lab 4)

int StreamToDevice=0;                // 0=UART, 1=stream to file (Lab 4)

int fputc (int ch, FILE *f) { 
  if(StreamToDevice==1){  // Lab 4
    if(eFile_Write(ch)){          // close file on error
       OS_EndRedirectToFile(); // cannot write to file
       return 1;                  // failure
    }
    return 0; // success writing
  }
  
  // default UART output
  UART_OutChar(ch);
  return ch;
}

int fgetc (FILE *f){
  char ch = UART_InChar();  // receive from keyboard
  UART_OutChar(ch);         // echo
  return ch;
}

int OS_RedirectToFile(const char *name){  // Lab 4
  eFile_Create(name);              // ignore error if file already exists
  if(eFile_WOpen(name)) return 1;  // cannot open file
  StreamToDevice = 1;
  return 0;
}

int OS_EndRedirectToFile(void){  // Lab 4
  StreamToDevice = 0;
  if(eFile_WClose()) return 1;    // cannot close file
  return 0;
}

int OS_RedirectToUART(void){
  StreamToDevice = 0;
  return 0;
}

int OS_RedirectToST7735(void){
  
  return 1;
}

uint32_t OS_NumThreads_Created() {
	return NumThreadsCreated;
}