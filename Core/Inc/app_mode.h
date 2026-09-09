#ifndef APP_MODE_H
#define APP_MODE_H

/* Flash-and-run test: home CyberGear once, then reciprocate. RobStride TX is
 * disabled and inter-board motion commands are ignored for the entire boot.
 * Set to 0 and rebuild to restore the normal multi-axis application. */
#ifndef APP_CYBERGEAR_STANDALONE_TEST
#define APP_CYBERGEAR_STANDALONE_TEST 1
#endif

#endif
