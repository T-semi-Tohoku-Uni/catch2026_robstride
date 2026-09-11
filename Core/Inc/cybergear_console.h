#ifndef CYBERGEAR_CONSOLE_H
#define CYBERGEAR_CONSOLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cybergear_config.h"

typedef enum {
    CG_CONSOLE_NONE, CG_CONSOLE_START, CG_CONSOLE_STOP, CG_CONSOLE_REINIT,
    CG_CONSOLE_SET, CG_CONSOLE_SHOW, CG_CONSOLE_HELP, CG_CONSOLE_INVALID
} CyberGearConsoleAction;

typedef struct {
    volatile uint32_t head, tail;
    volatile bool input_error, dropping, stop_requested, receiving_line;
    uint8_t received[CG_CONSOLE_RX_CAPACITY];
    char line[CG_CONSOLE_LINE_CAPACITY];
    size_t length;
    bool line_discard;
} CyberGearConsole;

typedef struct {
    CyberGearConsoleAction action;
    char key[CG_CONSOLE_LINE_CAPACITY];
    char value[CG_CONSOLE_LINE_CAPACITY];
} CyberGearConsoleCommand;

void cybergear_console_init(CyberGearConsole *console);
void cybergear_console_receive(CyberGearConsole *console, uint8_t byte);
void cybergear_console_receive_error(CyberGearConsole *console);
CyberGearConsoleAction cybergear_console_poll(CyberGearConsole *console,
    CyberGearConsoleCommand *command);

#endif
