#include <stdio.h>

int test_trajectory(void);
void test_controller(void);
void test_dynamics(void);
void test_driver(void);
void test_controller_simulation(void);

int main(void)
{
    test_trajectory();
    test_controller();
    test_dynamics();
    test_driver();
    test_controller_simulation();
    puts("CyberGear host tests: all checks passed (no hardware access).");
    return 0;
}
