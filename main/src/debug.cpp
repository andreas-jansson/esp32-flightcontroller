#include "debug.h"
#include "stdio.h"
#include "stdarg.h"


static volatile uint32_t logFile = 0;
static volatile uint32_t logPrio = 0;


void set_loglevel(uint32_t a_file, uint32_t a_prio){
    logFile = a_file;
    logPrio = a_prio;
}

void print_msg(enum DebugPrio a_prio, const char *format,  va_list args){


    if(a_prio & logPrio)
        vprintf(format, args);
}


void print_debug(enum DebugFileLevel a_file, enum DebugPrio a_prio, const char *format, ...){


    va_list args; // Variable argument list
    va_start(args, format);

    if(a_file & logFile){
        print_msg(a_prio, format, args);
    }
    va_end(args); // End the first use of args

 
}