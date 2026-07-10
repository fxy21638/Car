#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_CONTROL_PERIOD_MS             (10UL)
#define APP_OLED_REFRESH_MS               (100UL)
#define APP_IMU_MAX_CONSECUTIVE_FAILURES  (5U)
#define APP_BASE_SPEED                    (80)

#define APP_TRACE_PULSES_PER_CIRCLE       (17500U)
#define APP_AVOID_PULSES_PER_CIRCLE       (19000U)
#define APP_DIAG_PULSES_PER_CIRCLE        (21400U)

#define APP_AVOID_TURN_DEG                (45.0f)
#define APP_AVOID_RETURN_DEG              (45.0f)
#define APP_AVOID_DIST_PULSES             (1350)
#define APP_AVOID_SPEED                   (60)
#define APP_AVOID_OBSTACLE_CM             (25)
#define APP_AVOID_CONVERGE_DEG            (3.0f)
#define APP_AVOID_SEEK_MAX_PULSES         (2500)

#define APP_CORNER_TURN_DEG               (137.0f)
#define APP_CORNER_STRAIGHT_PULSES        (6100)
#define APP_CORNER_STRAIGHT_SPEED         (80)
#define APP_CORNER_ADVANCE_PULSES         (200)
#define APP_CORNER_RETURN_PULSES          (300)
#define APP_CORNER_CONVERGE_DEG           (3.0f)
#define APP_CORNER_DETECT_DEBOUNCE        (3)

#define APP_TURN_TIMEOUT_MS                (3000UL)
#define APP_ADVANCE_TIMEOUT_MS             (3000UL)
#define APP_FORWARD_TIMEOUT_MS             (5000UL)
#define APP_SEEK_TIMEOUT_MS                (5000UL)
#define APP_STRAIGHT_TIMEOUT_MS            (8000UL)

#define APP_ENCODER_STALL_TIMEOUT_MS       (500UL)
#define APP_ENCODER_STALL_MIN_COMMAND      (20)

#define APP_WATCHDOG_SERVICE_MS            (250UL)

#endif
