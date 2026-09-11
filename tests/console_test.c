#include "cybergear_console.h"
#include <assert.h>
#include <string.h>

static void receive_text(CyberGearConsole *console, const char *text)
{
    while (*text != '\0') cybergear_console_receive(console, (uint8_t)*text++);
}

void console_tests(void)
{
    CyberGearConsole console;
    CyberGearConsoleCommand command;
    cybergear_console_init(&console);
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_NONE);
    receive_text(&console, "s");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_NONE);
    receive_text(&console, "et current 2.5\r\n");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_SET);
    assert(strcmp(command.key, "current") == 0 && strcmp(command.value, "2.5") == 0);
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_NONE);
    receive_text(&console, "s\nS\r\nreinit\nshow\nhelp\n?\n");
    const CyberGearConsoleAction expected[] = {
        CG_CONSOLE_START, CG_CONSOLE_START, CG_CONSOLE_REINIT,
        CG_CONSOLE_SHOW, CG_CONSOLE_HELP, CG_CONSOLE_HELP};
    for (unsigned int index = 0U; index < sizeof(expected) / sizeof(expected[0]); ++index)
        assert(cybergear_console_poll(&console, &command) == expected[index]);
    receive_text(&console, "s trailing\nset current 2 more\nset current\nreinit more\n");
    for (unsigned int index = 0U; index < 4U; ++index)
        assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_INVALID);
    receive_text(&console, "set\twc\t3\n");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_SET);
    assert(strcmp(command.key, "wc") == 0);
    receive_text(&console, "set current ");
    cybergear_console_receive_error(&console);
    receive_text(&console, "s\n");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_INVALID);
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_NONE);
    for (unsigned int index = 0U; index < CG_CONSOLE_RX_CAPACITY + 10U; ++index)
        cybergear_console_receive(&console, 'a');
    receive_text(&console, "s\n");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_INVALID);
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_NONE);
    for (unsigned int index = 0U; index < CG_CONSOLE_LINE_CAPACITY + 5U; ++index) {
        cybergear_console_receive(&console, 'a');
        assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_NONE);
    }
    receive_text(&console, "s\n");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_INVALID);
    cybergear_console_receive(&console, 0U);
    receive_text(&console, "s\n");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_INVALID);
    receive_text(&console, "set current 2.5\nx");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_STOP);
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_NONE);
    cybergear_console_receive_error(&console);
    receive_text(&console, "X");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_STOP);
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_INVALID);
    receive_text(&console, "\ns\n");
    assert(cybergear_console_poll(&console, &command) == CG_CONSOLE_START);
}
