// *************Interpreter.c**************
// Students implement this as part of EE445M/EE380L.12 Lab 1,2,3,4 
// High-level OS user interface
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano 1/18/20, valvano@mail.utexas.edu
#include <stdint.h>
#include <string.h> 
#include <stdio.h>
#include <stdlib.h>
#include "../RTOS_Labs_common/OS.h"
#include "../RTOS_Labs_common/ST7735.h"
#include "../inc/ADCT0ATrigger.h"
#include "../inc/ADCSWTrigger.h"
#include "../RTOS_Labs_common/UART0int.h"
#include "../RTOS_Labs_common/eDisk.h"
#include "../RTOS_Labs_common/eFile.h"
#include "../RTOS_Labs_common/ADC.h"

#define MAX_USER_INPUT_LENGTH 100 // Number of characters a user can input
#define MAX_COMMAND_LENGTH 5 	    // Number of characters in a command
#define MAX_COMMANDS 1				    // Number of commands at a time
#define MAX_ARGS 10						    // Number of arguments in a command
#define MAX_ARG_LENGTH 10			    // Number of characters per argument

#define SUCCESS 1
#define FAILURE 0

extern uint32_t MaxJitter;
extern uint32_t int_max_disable_time;

// #define DEBUG

// Print jitter histogram
void Jitter(int32_t MaxJitter, uint32_t const JitterSize, uint32_t JitterHistogram[]){
  // write this for Lab 3 (the latest)
	UART_OutString("MaxJitter = ");
  UART_OutUDec(MaxJitter);
  UART_OutString("\n\r");
  UART_OutChar('[');
  for (int i = 0; i < JitterSize; i++) {
    UART_OutUDec(JitterHistogram[i]);
    UART_OutChar(' ');
  }
  UART_OutChar(']');
  UART_OutString("\n\r");
}

typedef struct command {
	char command[MAX_COMMAND_LENGTH];
	char args[MAX_ARGS][MAX_ARG_LENGTH];
	int param_size;
} command_struct;

/* Commands accepted by the interpreter */
int8_t Help_Command(void);
int8_t LCD_Command(command_struct* command);
int8_t ADC_Command(command_struct* command);
int8_t Timer_Command(command_struct* command);
int8_t GPIO_Command(command_struct* command);
int8_t Num_created_Command(command_struct * command);
int8_t MaxJitter_Command(command_struct* command);
int8_t InterruptPercentage_Command(command_struct* command);
int8_t MaxInterrupt_Command(command_struct* command);

command_struct command_array[MAX_COMMANDS];

void Parse_Command_Into_Struct (char* full_command, 
						command_struct* command_struct_to_fill) {
	char sep[2] = " ";
  char* token = strtok(full_command, sep);                             /* points token and command to the same string in mem */
  strncpy(command_struct_to_fill->command, token, MAX_COMMAND_LENGTH); /* strncpy prevents going into oob memory */
  int arg = 0;
  while (token != NULL) {
    if (arg == MAX_ARGS) break;                                        /* prevents going into oob memory */
    token = strtok(NULL, sep);
    strncpy(command_struct_to_fill->args[arg], token, MAX_ARG_LENGTH);
    ++arg;
  }
  command_struct_to_fill->param_size = arg - 1; /* POSSIBLE BUG WITH CARRIAGE RETURNS */
}

// *********** Command line interpreter (shell) ************
void Interpreter(void){
	while(1) {
    char inString[MAX_USER_INPUT_LENGTH];
		UART_OutString("\n\r> ");                                 /* prompt */
		UART_InString(inString, MAX_USER_INPUT_LENGTH);           /* wait for user input */
		UART_OutString("\n\r");
    Parse_Command_Into_Struct(inString, command_array);       /* parse user input into a command_struct */
		
    #ifdef DEBUG
    UART_OutString(command_array[0].command);
    for (int i = 0; i < command_array[0].param_size; i++) {
      UART_OutString(command_array[0].args[i]);
    }
    #endif
		
		uint8_t command_success_status;
		if(strcmp(command_array[0].command, "lcd")==0) {
			command_success_status = LCD_Command(&command_array[0]);
		} else if (strcmp(command_array[0].command, "time")==0) {
			command_success_status = Timer_Command(&command_array[0]);
		} else if (strcmp(command_array[0].command, "adc")==0) {
			command_success_status = ADC_Command(&command_array[0]);
		} else if (strcmp(command_array[0].command, "gpio") == 0) {
      command_success_status = GPIO_Command(&command_array[0]);
    } else if (strcmp(command_array[0].command, "help")==0) {
      command_success_status = Help_Command();
    } else if (strcmp(command_array[0].command, "numt") == 0) {
			command_success_status = Num_created_Command(&command_array[0]);
		} else if (strcmp(command_array[0].command, "mj") == 0) {
      command_success_status = MaxJitter_Command(&command_array[0]);
    } else if (strcmp(command_array[0].command, "int%") == 0) {
			command_success_status = InterruptPercentage_Command(&command_array[0]);
		} else if (strcmp(command_array[0].command, "mint") == 0) {
      command_success_status = MaxInterrupt_Command(&command_array[0]);
    }else {
			// Invalid command -> error
			command_success_status = FAILURE;
		}
		
		if(command_success_status == FAILURE) {
			UART_OutString("Command failed.\n\r");
		}
	}
}

/*  LCD_Command
 *  Outputs a user defined string to a user specified
 *  line on the top or bottom of the ST7735. The command
 *  has the following format:
 *    lcd t/b line_number user_string
 *  where line_number is an integer in range [0, 7].
 *  Input:
 *    - command (command_struct*): pointer to a user command
 *  Output (int8_t):
 *    - The command's SUCCESS or FAILURE status 
 */
int8_t LCD_Command(command_struct* command) {
  if(command->param_size < 3) {
    UART_OutString("Missing arguments. Correct format:\n\r");
    UART_OutString("lcd t/b line_number user_string\n\r");
    return FAILURE;
  }
  
  uint8_t device;
  uint8_t line;
  char buf[MAX_ARGS*MAX_ARG_LENGTH];                         /* ensure that buf has enough space to hold the user input string */
  memset(buf, 0, sizeof(buf));
  if (!strcmp(command->args[0], "t")) {                      /* which device should be printed to */
    device = 0;
  }
  else if (!strcmp(command->args[0], "b")) {
    device = 1;
  }
  else {
    UART_OutString("The first argument must be t or b.\n\r");
    return FAILURE;
  }
  line = atoi(command->args[1]);                        /* convert user line_number to integer (0 if user's input cannot convert) */
  for (int i = 2; i < command->param_size; i++) {
    strncat(buf, command->args[i], MAX_ARG_LENGTH);
    strncat(buf, " ", 2);
  }
  ST7735_Message(device, line, buf, INT32_MIN);
  return SUCCESS;
}

int8_t ADC_Command(command_struct* command) {
  uint32_t sample = ADC_In();
	UART_OutUDec(sample);
	UART_OutString("\n\r");
	ST7735_Message(0, 0, "ADC Sample =", sample);
	return SUCCESS;
}

int8_t Timer_Command(command_struct* command) {
	/* Look at 2nd argument and perform respective command */
	if(command->param_size <1) {
		UART_OutString("Missing arg \n\r");
		return FAILURE; //Not entered properly, need second argument
	} else {
		if(command->args[0][0] == 's' 
				&& command->args[0][1] == 't') {
					OS_StartMsTime();
		} else if (command->args[0][0] == 's' 
				&& command->args[0][1] == 'p') {
					OS_StopMsTime();
		} else if (command->args[0][0] == 'g' 
				&& command->args[0][1] == 't') {
          uint32_t time = OS_MsTime();
					ST7735_Message(0, 0, "OS Time=", time/10);
		} else if (command->args[0][0] == 'c' 
				&& command->args[0][1] == 'l') {
					OS_ClearMsTime();
		} else {
			// Incorrect arugment
			UART_OutString("Incorrect arg \n\r");
			return FAILURE;
		}
	}
	return SUCCESS;
}

/*  GPIO_Command
 *  Updates a user specified GPIO pin. The command
 *  has the following format:
 *     gpio port_letter pin_number 0/1/2
 *  where the 0/1/2 option stands for low/high/toggle respectively.
 *  Input:
 *    - command (command_struct*): pointer to a user command
 *  Output (int8_t):
 *    - The command's SUCCESS or FAILURE status 
 */
int8_t GPIO_Command(command_struct* command) {
  if (command->param_size != 3) {
    UART_OutString("Missing parameters. help for info\n\r");
    return FAILURE;
  }
  
  if (strlen(command->args[0]) > 1) {
    UART_OutString("The port letter must be length one.\n\r");
    return FAILURE;
  }
  volatile uint32_t* pin_address;
  uint8_t port = command->args[0][0];
  uint8_t pin = command->args[1][0] - 0x30;
  
  switch (port) {
    case 'f':
      if (pin > 4) {
        UART_OutString("The specified port does not have that many pins.\n\r");
        return FAILURE;
      }
      pin_address = (volatile uint32_t*) (0x40025000 + (0x0000004 << pin));
      break;
    default:
      UART_OutString("This port is not currently implemented.\n\r");
  }
  
  *pin_address ^= 0x01 << pin;
  return SUCCESS;
}

int8_t Num_created_Command(command_struct * command) {
	UART_OutString("Num created: ");
	UART_OutUDec(OS_NumThreads_Created());
  ST7735_Message(0, 0, "Num Threads =", OS_NumThreads_Created());
  return SUCCESS;
}

int8_t MaxJitter_Command(command_struct* command) {
  ST7735_Message(0, 0, "Max Jitter =", MaxJitter);
  return SUCCESS;
}

int8_t InterruptPercentage_Command(command_struct* command) {
	long percentage = get_int_percentage_time();
	UART_OutUDec(percentage / 100000);
	UART_OutChar('.');
	percentage %= 100;
	UART_OutUDec(percentage);
  return SUCCESS;
}

int8_t MaxInterrupt_Command(command_struct* command) {
  UART_OutUDec(int_max_disable_time);
  UART_OutString(" [No. clock cycles]");
  return SUCCESS;
}

int8_t Help_Command() {
  // LCD COMMAND
  UART_OutString("lcd t/b line_number user_string\n\r"
                 "  - Prints user string to top (t) or bottom (b) LCD device on specified line.\n\n\r");
  // ADC COMMAND
  UART_OutString("adc\n\r"
                 "  - Prints an ADC sample.\n\n\r");
  // TIME COMMAND
  UART_OutString("time st/sp/gt/cl\n\r"
                 "  - 'st' start timer\n\r"
                 "  - 'sp' stop timer\n\r"
                 "  - 'gt' get time\n\r"
                 "  - 'cl' clear timer\n\n\r");
  // GPIO COMMAND
  UART_OutString("gpio port_letter pin_number 0/1/2\n\r"
                 "  - Alters the specified port and pin. Does not currently initialize the port.\n\r"
                 "  - 0/1/2 correspond to low/high/toggle\n\n\r");
  
  // NUM THREADS COMMAND
  UART_OutString("numt\n\r"
                 "  - Print the number of created threads.\n\r\n\r");
  
  // MAX JITTER COMMAND
  UART_OutString("mj\n\r"
                 "  - Print the maximum jitter measured in the system.\n\n\r");
                 
  // INTERRUPT PERCENTAGE COMMAND
  UART_OutString("int%\n\r"
                 "  - Print the percentage of time interrupts have been disabled.\n\n\r");
                 
  // MAX INTERRUPT COMMAND
  UART_OutString("mint\n\r"
                 "  - Print the time interrupts have been disabled for.\n\n\r");
                 
  return SUCCESS;
}

