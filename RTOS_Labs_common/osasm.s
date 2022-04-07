;/*****************************************************************************/
;/* OSasm.s: low-level OS commands, written in assembly                       */
;/* derived from uCOS-II                                                      */
;/*****************************************************************************/
;Jonathan Valvano, OS Lab2/3/4/5, 1/12/20
;Students will implement these functions as part of EE445M/EE380L.12 Lab

        AREA |.text|, CODE, READONLY, ALIGN=2
        THUMB
        REQUIRE8
        PRESERVE8

        EXTERN  current_thread_ptr            ; currently running thread

        EXPORT  StartOS
        EXPORT  ContextSwitch
        EXPORT  PendSV_Handler
        EXPORT  SVC_Handler


NVIC_INT_CTRL   EQU     0xE000ED04                              ; Interrupt control state register.
NVIC_SYSPRI14   EQU     0xE000ED22                              ; PendSV priority register (position 14).
NVIC_SYSPRI15   EQU     0xE000ED23                              ; Systick priority register (position 15).
NVIC_LEVEL14    EQU           0xC0                              ; Systick priority value (second lowest).
NVIC_LEVEL15    EQU           0xE0                              ; PendSV priority value (lowest).
NVIC_PENDSVSET  EQU     0x10000000                              ; Value to trigger PendSV exception.


StartOS
	LDR R0, =NVIC_SYSPRI14			; Setting PendSV priority to 7
	LDR R2, =NVIC_LEVEL15
	STRB R2, [R0]
	
	LDR R0, =NVIC_SYSPRI15			; Setting Systick priority to 6
	LDR R2, =NVIC_LEVEL14
	STRB R2, [R0]
	
    LDR R0, =current_thread_ptr ; getting the memory address of current_thread_ptr (**tcb)
    LDR R1, [R0]                ; getting the pointer to the tcb (*tcb)
    LDR SP, [R1]                ; getting the actual tcb
    POP {R4-R11}                
    POP {R0-R3}
    POP {R12}
    POP {LR}                    ; discard link register
    POP {LR}                    ; link to start of thread
    POP {R1}                    ; discard PSR
    CPSIE   I                   ; enable interrupts
    BX      LR                  ; start first thread

OSStartHang
    B       OSStartHang        ; Should never get here


;********************************************************************************************************
;                               PERFORM A CONTEXT SWITCH (From task level)
;                                           void ContextSwitch(void)
;
; Note(s) : 1) ContextSwitch() is called when OS wants to perform a task context switch.  This function
;              triggers the PendSV exception which is where the real work is done.
;********************************************************************************************************

ContextSwitch
	LDR R0, =NVIC_INT_CTRL 
	LDR R1, =NVIC_PENDSVSET
	STR R1, [R0]
	BX LR
    

;********************************************************************************************************
;                                         HANDLE PendSV EXCEPTION
;                                     void OS_CPU_PendSVHandler(void)
;
; Note(s) : 1) PendSV is used to cause a context switch.  This is a recommended method for performing
;              context switches with Cortex-M.  This is because the Cortex-M3 auto-saves half of the
;              processor context on any exception, and restores same on return from exception.  So only
;              saving of R4-R11 is required and fixing up the stack pointers.  Using the PendSV exception
;              this way means that context saving and restoring is identical whether it is initiated from
;              a thread or occurs due to an interrupt or exception.
;
;           2) Pseudo-code is:
;              a) Get the process SP, if 0 then skip (goto d) the saving part (first context switch);
;              b) Save remaining regs r4-r11 on process stack;
;              c) Save the process SP in its TCB, OSTCBCur->OSTCBStkPtr = SP;
;              d) Call OSTaskSwHook();
;              e) Get current high priority, OSPrioCur = OSPrioHighRdy;
;              f) Get current ready thread TCB, OSTCBCur = OSTCBHighRdy;
;              g) Get new process SP from TCB, SP = OSTCBHighRdy->OSTCBStkPtr;
;              h) Restore R4-R11 from new process stack;
;              i) Perform exception return which will restore remaining context.
;
;           3) On entry into PendSV handler:
;              a) The following have been saved on the process stack (by processor):
;                 xPSR, PC, LR, R12, R0-R3
;              b) Processor mode is switched to Handler mode (from Thread mode)
;              c) Stack is Main stack (switched from Process stack)
;              d) OSTCBCur      points to the OS_TCB of the task to suspend
;                 OSTCBHighRdy  points to the OS_TCB of the task to resume
;
;           4) Since PendSV is set to lowest priority in the system (by OSStartHighRdy() above), we
;              know that it will only be run when no other exception or interrupt is active, and
;              therefore safe to assume that context being switched out was using the process stack (PSP).
;********************************************************************************************************

PendSV_Handler
    MRS     R3, PRIMASK                ; Start critical section
    CPSID   I                          ; 2) Prevent interrupt during switch
    PUSH    {R4-R11}                   ; 3) Save remaining regs r4-11
    LDR     R0, =current_thread_ptr    ; 4) R0=pointer to RunPt, old thread
    LDR     R1, [R0]                   ;    R1 = RunPt
    STR     SP, [R1]                   ; 5) Save SP into TCB
    LDR     R2, [R1,#8]                ; Load flag to check for priority jump
    CMP     R2, #1                     ; If true then jump priorites
    BEQ     Jump
    LDR     R1, [R1,#4]                ; 6) R1 = RunPt->next
    B Done
Jump
    MOV     R2, #0
    STR     R2, [R1,#8]				   ; Clear jump flag
    LDR     R1, [R1,#12]               ; R1 = RunPt->jumpTo
Done
    STR     R1, [R0]                   ;    RunPt = R1
    LDR     SP, [R1]                   ; 7) new thread SP; SP = RunPt->sp;
    POP     {R4-R11}                   ; 8) restore regs r4-11
    MSR     PRIMASK, R3                ; End critical section
    BX      LR                         ; Exception return will restore remaining context   
    

;********************************************************************************************************
;                                         HANDLE SVC EXCEPTION
;                                     void OS_CPU_SVCHandler(void)
;
; Note(s) : SVC is a software-triggered exception to make OS kernel calls from user land. 
;           The function ID to call is encoded in the instruction itself, the location of which can be
;           found relative to the return address saved on the stack on exception entry.
;           Function-call paramters in R0..R3 are also auto-saved on stack on exception entry.
;********************************************************************************************************

        IMPORT    OS_Id
        IMPORT    OS_Kill
        IMPORT    OS_Sleep
        IMPORT    OS_Time
        IMPORT    OS_AddThread

SVC_Handler
; put your Lab 5 code here


    BX      LR                   ; Return from exception



    ALIGN
    END
