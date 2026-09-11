#ifndef RS03_HOMING_CONFIG_H
#define RS03_HOMING_CONFIG_H

/* Externally driven inputs; existing board GPIO configuration has no pull. */
#define RS03_HOME_RIGHT_PORT limit_sw_0_GPIO_Port
#define RS03_HOME_RIGHT_PIN limit_sw_0_Pin
#define RS03_HOME_ACTIVE_STATE GPIO_PIN_SET

/* Motor coordinates: positive = CCW viewed from the output shaft. */
#define RS03_HOME_RIGHT_SPEED_RAD_S (-0.2f)
#define RS03_HOME_CURRENT_LIMIT_A 2.0f
#define RS03_HOME_TIMEOUT_MS 15000U
#define RS03_HOME_FEEDBACK_TIMEOUT_MS 500U
#define RS03_HOME_SETTLE_MS 200U
#define RS03_HOME_POLL_MS 10U

#endif
