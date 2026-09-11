#include "cybergear_console.h"
#include <string.h>

void cybergear_console_init(CyberGearConsole *console)
{
    memset(console, 0, sizeof(*console));
}

void cybergear_console_receive_error(CyberGearConsole *console)
{
    console->input_error = true;
    console->dropping = true;
    console->receiving_line = true;
}

void cybergear_console_receive(CyberGearConsole *console, uint8_t byte)
{
    if (byte == CG_TEST_STOP_COMMAND || byte == CG_TEST_STOP_COMMAND_UPPER) {
        console->stop_requested = true;
        return;
    }
    console->receiving_line = byte != '\r' && byte != '\n';
    if (console->dropping) {
        if (byte == '\r' || byte == '\n') console->dropping = false;
        return;
    }
    const uint32_t next = (console->head + 1U) % CG_CONSOLE_RX_CAPACITY;
    if (next == console->tail) {
        cybergear_console_receive_error(console);
        if (byte == '\r' || byte == '\n') {
            console->dropping = false;
            console->receiving_line = false;
        }
        return;
    }
    console->received[console->head] = byte;
    console->head = next;
}

static CyberGearConsoleAction parse_line(char *line, CyberGearConsoleCommand *command)
{
    char *tokens[4] = {0};
    size_t count = 0U;
    char *cursor = line;
    while (*cursor != '\0') {
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (*cursor == '\0') break;
        if (count == 4U) return CG_CONSOLE_INVALID;
        tokens[count++] = cursor;
        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') cursor++;
        if (*cursor != '\0') *cursor++ = '\0';
    }
    if (count == 0U) return CG_CONSOLE_NONE;
    if (count == 1U) {
        if (tokens[0][1] == '\0' &&
            (tokens[0][0] == CG_TEST_START_COMMAND || tokens[0][0] == CG_TEST_START_COMMAND_UPPER))
            return CG_CONSOLE_START;
        if (strcmp(tokens[0], "reinit") == 0) return CG_CONSOLE_REINIT;
        if (strcmp(tokens[0], "show") == 0) return CG_CONSOLE_SHOW;
        if (strcmp(tokens[0], "help") == 0 || strcmp(tokens[0], "?") == 0) return CG_CONSOLE_HELP;
    }
    if (count == 3U && strcmp(tokens[0], "set") == 0) {
        strcpy(command->key, tokens[1]);
        strcpy(command->value, tokens[2]);
        return CG_CONSOLE_SET;
    }
    return CG_CONSOLE_INVALID;
}

CyberGearConsoleAction cybergear_console_poll(CyberGearConsole *console,
    CyberGearConsoleCommand *command)
{
    command->action = CG_CONSOLE_NONE;
    if (console->stop_requested) {
        console->stop_requested = false;
        console->tail = console->head;
        console->length = 0U;
        console->line_discard = false;
        command->action = CG_CONSOLE_STOP;
        return command->action;
    }
    if (console->input_error) {
        console->input_error = false;
        console->tail = console->head;
        console->length = 0U;
        console->line_discard = false;
        command->action = CG_CONSOLE_INVALID;
        return command->action;
    }
    while (console->tail != console->head) {
        const uint8_t byte = console->received[console->tail];
        console->tail = (console->tail + 1U) % CG_CONSOLE_RX_CAPACITY;
        if (byte == '\r' || byte == '\n') {
            if (console->line_discard) command->action = CG_CONSOLE_INVALID;
            else {
                console->line[console->length] = '\0';
                command->action = parse_line(console->line, command);
            }
            console->length = 0U;
            console->line_discard = false;
            if (command->action != CG_CONSOLE_NONE) return command->action;
        } else if (byte < 32U || byte > 126U || console->length + 1U >= sizeof(console->line)) {
            if (byte == '\t' && console->length + 1U < sizeof(console->line) && !console->line_discard)
                console->line[console->length++] = (char)byte;
            else console->line_discard = true;
        } else if (!console->line_discard) console->line[console->length++] = (char)byte;
    }
    return command->action;
}
