#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "../inc/TCB.h"

struct tcb tcb_memory_pool[TOTAL_TCB];
uint32_t tcb_memory_management;

int32_t stack_memory[TOTAL_TCB][STACK_SIZE_PER_THREAD];


void tcb_mem_init() {
	tcb_memory_management = 0;
}

struct tcb* tcb_mem_get_tcb(void) {
	uint8_t index = 0;
	uint8_t found = 0;
	while(index < 32 && !found) {
		if(!(tcb_memory_management & (0x01 << index))) {
			found = 1;
		}
    else {
      index++;
    }
	}
	/* Out of memory */
	if(!found || index >= TOTAL_TCB) {
		return NULL;
	}
	/* Return TCB */
	tcb_memory_management |= (0x01 << index);
	tcb_memory_pool[index].pool_index = index;
	tcb_memory_pool[index].stack_pointer = (uint32_t *)stack_memory[index] + STACK_SIZE_PER_THREAD;
	return &tcb_memory_pool[index];
}

uint8_t tcb_mem_free_tcb(struct tcb* to_free) {
	/* Take index and mark as free */
	uint32_t index = to_free->pool_index;
	tcb_memory_management &= ~(0x01 << index);
}