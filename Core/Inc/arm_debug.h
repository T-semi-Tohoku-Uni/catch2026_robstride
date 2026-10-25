#ifndef ARM_DEBUG_H
#define ARM_DEBUG_H

/* Read-only live angles; fault records in the status log remain frozen. */
static void arm_debug_angles_poll(void)
{
  static uint32_t last_ms;
  static uint8_t last_complete = 255U, last_failed = 255U;
  struct {
    uint8_t status[12];
    float motor_rad, target_rad;
  } axes[3];

  if (__get_IPSR() || __get_PRIMASK()) return;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  const uint32_t now = HAL_GetTick();
  const uint8_t complete = arm_boot_complete, failed = arm_boot_failed;
  if (last_complete == complete && last_failed == failed &&
      (uint32_t)(now - last_ms) < 200U) {
    __set_PRIMASK(mask);
    return;
  }
  const uint8_t stage = arm_initialization_requested ? arm_boot_step : 31U;
  const uint8_t state = rpc_state;
  const uint32_t target_token = arm_target_token;
  for (unsigned i = 0; i < 3U; ++i) {
    arm_axis_record(axes[i].status, i, now);
    axes[i].motor_rad = i == 0U ? cybergear_base.feedback.position_rad :
      robstride_handler[i == 1U ? LEFT_RS03_INDEX : RIGHT_RS03_INDEX].feedback.position_rad;
    axes[i].target_rad = target_angle[i];
  }
  __set_PRIMASK(mask);
  last_ms = now;
  last_complete = complete;
  last_failed = failed;

  /* Format only after IRQs resume; never send CAN requests for diagnostics. */
  printf("angles_live t_ms=%lu stage=%u boot_complete=%u rpc=%u target_token=%lu\r\n",
         (unsigned long)now, (unsigned)stage, (unsigned)complete, (unsigned)state,
         (unsigned long)target_token);
  for (unsigned i = 0; i < 3U; ++i) {
    const uint8_t *a = axes[i].status;
    const unsigned valid = (a[2] & 1U) && isfinite(axes[i].motor_rad);
    const float motor = valid ? axes[i].motor_rad : NAN;
    /* Match the existing current-feedback ROS coordinate conversion. */
    const float joint = i == 0U ? motor :
      (i == 1U ? -(motor + 1.0) : motor + 1.884);
    printf("angle axis=%u id=%u mode=%u valid=%u fresh=%u age_ms=%lu "
           "motor_rad=%+.4f joint_rad=%+.4f target_rad=%+.4f\r\n",
           i, a[3], a[0], valid, (unsigned)(valid && (a[2] & 2U)),
           (unsigned long)canrpc_get_u32(a + 4), (double)motor, (double)joint,
           (double)axes[i].target_rad);
  }
}

#endif
