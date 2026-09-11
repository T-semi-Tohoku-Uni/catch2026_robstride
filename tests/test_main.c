#include <stdio.h>
#include <string.h>
#ifdef _MSC_VER
#include <stdlib.h>
#endif

int test_trajectory(void);
void test_controller(void);
void test_dynamics(void);
void test_driver(void);
void test_saturation(void);
void test_bus_monitoring(void);
void test_startup_handoff(void);
void test_controller_simulation(void);
void test_motion(void);
void tuning_tests(void);
void console_tests(void);

int main(int argc, char **argv)
{
#ifdef _MSC_VER
    /* Report failed assertions to CTest rather than opening a modal dialog. */
    _set_error_mode(_OUT_TO_STDERR);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
    if (argc == 2 && strcmp(argv[1], "saturation") == 0) {
        test_controller();
        test_saturation();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "bus") == 0) {
        test_bus_monitoring();
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "startup") == 0) {
        test_startup_handoff();
        return 0;
    }
    test_trajectory();
    test_controller();
    test_dynamics();
    test_driver();
    test_controller_simulation();
    test_motion();
    tuning_tests();
    console_tests();
    puts("CyberGear host tests: all checks passed (no hardware access).");
    return 0;
}
