#ifndef __TCB_H__
#define __TCB_H__

#include <stdint.h>


#define TOTAL_TCB 10
#define STACK_SIZE_PER_THREAD 128 //32 bit words

typedef struct tcb {
	uint32_t* stack_pointer;
	
	struct tcb* next;
  uint32_t jumpFlag;
  struct tcb* jumpTo;
	struct tcb* previous;

	uint8_t orig_priority;
	uint8_t aged_priority;
	uint32_t ticks;
	
	/* Adding to sleep list */
	struct tcb* sleep_next;
	uint32_t sleep_count;
	
	/* Semaphore indexing */
	struct tcb* sema_next;
	
	/* Thread specific data */
	unsigned int thread_id: 10;
	unsigned int blocked: 1;
	unsigned int pool_index;
	
} tcb;

void tcb_mem_init(void);
struct tcb* tcb_mem_get_tcb(void);
uint8_t tcb_mem_free_tcb(struct tcb* to_free);

#endif