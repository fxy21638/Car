/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     *  ======== SYSCFG_DL_init ========
     *  Perform all required MSP DL initialization
     *
     *  This function should be called once at a point before any use of
     *  MSP DL.
     */

    /* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                        TIMG12
#define PWM_0_INST_IRQHandler                                  TIMG12_IRQHandler
#define PWM_0_INST_INT_IRQN                                    (TIMG12_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_20
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM48)
#define GPIO_PWM_0_C0_IOMUX_FUNC                    IOMUX_PINCM48_PF_TIMG12_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_25
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM55)
#define GPIO_PWM_0_C1_IOMUX_FUNC                    IOMUX_PINCM55_PF_TIMG12_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for CAPTURE_0 */
#define CAPTURE_0_INST                                                   (TIMA1)
#define CAPTURE_0_INST_IRQHandler                               TIMA1_IRQHandler
#define CAPTURE_0_INST_INT_IRQN                                 (TIMA1_INT_IRQn)
#define CAPTURE_0_INST_LOAD_VALUE                                         (155U)
/* GPIO defines for channel 0 */
#define GPIO_CAPTURE_0_C0_PORT                                             GPIOA
#define GPIO_CAPTURE_0_C0_PIN                                      (DL_GPIO_PIN_6)
#define GPIO_CAPTURE_0_C0_IOMUX                                  (IOMUX_PINCM11)
#define GPIO_CAPTURE_0_C0_IOMUX_FUNC                 IOMUX_PINCM11_PF_TIMA1_CCP0





/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA0)
#define TIMER_0_INST_IRQHandler                                 TIMA0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (199U)
/* Defines for TIMER_US */
#define TIMER_US_INST                                                    (TIMG6)
#define TIMER_US_INST_IRQHandler                                TIMG6_IRQHandler
#define TIMER_US_INST_INT_IRQN                                  (TIMG6_INT_IRQn)
#define TIMER_US_INST_LOAD_VALUE                                           (49U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART1
#define UART_0_INST_IRQHandler                                  UART1_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                         DL_GPIO_PIN_9
#define GPIO_UART_0_TX_PIN                                         DL_GPIO_PIN_8
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM20)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM19)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM19_PF_UART1_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_0_FBRD_32_MHZ_9600_BAUD                                        (21)





/* Port definition for Pin Group LEDB */
#define LEDB_PORT                                                        (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LEDB_PIN_22_PIN                                         (DL_GPIO_PIN_22)
#define LEDB_PIN_22_IOMUX                                        (IOMUX_PINCM50)
/* Port definition for Pin Group TRACK_SENSOR */
#define TRACK_SENSOR_PORT                                                (GPIOB)

/* Defines for S0: GPIOB.23 with pinCMx 51 on package pin 22 */
#define TRACK_SENSOR_S0_PIN                                     (DL_GPIO_PIN_23)
#define TRACK_SENSOR_S0_IOMUX                                    (IOMUX_PINCM51)
/* Defines for S1: GPIOB.21 with pinCMx 49 on package pin 20 */
#define TRACK_SENSOR_S1_PIN                                     (DL_GPIO_PIN_21)
#define TRACK_SENSOR_S1_IOMUX                                    (IOMUX_PINCM49)
/* Defines for S2: GPIOB.19 with pinCMx 45 on package pin 16 */
#define TRACK_SENSOR_S2_PIN                                     (DL_GPIO_PIN_19)
#define TRACK_SENSOR_S2_IOMUX                                    (IOMUX_PINCM45)
/* Defines for S3: GPIOB.18 with pinCMx 44 on package pin 15 */
#define TRACK_SENSOR_S3_PIN                                     (DL_GPIO_PIN_18)
#define TRACK_SENSOR_S3_IOMUX                                    (IOMUX_PINCM44)
/* Defines for S4: GPIOB.17 with pinCMx 43 on package pin 14 */
#define TRACK_SENSOR_S4_PIN                                     (DL_GPIO_PIN_17)
#define TRACK_SENSOR_S4_IOMUX                                    (IOMUX_PINCM43)
/* Defines for S5: GPIOB.16 with pinCMx 33 on package pin 4 */
#define TRACK_SENSOR_S5_PIN                                     (DL_GPIO_PIN_16)
#define TRACK_SENSOR_S5_IOMUX                                    (IOMUX_PINCM33)
/* Defines for S6: GPIOB.15 with pinCMx 32 on package pin 3 */
#define TRACK_SENSOR_S6_PIN                                     (DL_GPIO_PIN_15)
#define TRACK_SENSOR_S6_IOMUX                                    (IOMUX_PINCM32)
/* Defines for S7: GPIOB.9 with pinCMx 26 on package pin 61 */
#define TRACK_SENSOR_S7_PIN                                      (DL_GPIO_PIN_9)
#define TRACK_SENSOR_S7_IOMUX                                    (IOMUX_PINCM26)
/* Port definition for Pin Group MOTOR */
#define MOTOR_PORT                                                       (GPIOB)

/* Defines for AIN1: GPIOB.12 with pinCMx 29 on package pin 64 */
#define MOTOR_AIN1_PIN                                          (DL_GPIO_PIN_12)
#define MOTOR_AIN1_IOMUX                                         (IOMUX_PINCM29)
/* Defines for AIN2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define MOTOR_AIN2_PIN                                          (DL_GPIO_PIN_13)
#define MOTOR_AIN2_IOMUX                                         (IOMUX_PINCM30)
/* Defines for BIN1: GPIOB.1 with pinCMx 13 on package pin 48 */
#define MOTOR_BIN1_PIN                                           (DL_GPIO_PIN_1)
#define MOTOR_BIN1_IOMUX                                         (IOMUX_PINCM13)
/* Defines for BIN2: GPIOB.4 with pinCMx 17 on package pin 52 */
#define MOTOR_BIN2_PIN                                           (DL_GPIO_PIN_4)
#define MOTOR_BIN2_IOMUX                                         (IOMUX_PINCM17)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOA)

/* Defines for E1A: GPIOA.15 with pinCMx 37 on package pin 8 */
// groups represented: ["ENCODERB","ENCODERA"]
// pins affected: ["E2A","E2B","E1A","E1B"]
#define GPIO_MULTIPLE_GPIOA_INT_IRQN                            (GPIOA_INT_IRQn)
#define GPIO_MULTIPLE_GPIOA_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO15)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_15)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM37)
/* Defines for E1B: GPIOA.14 with pinCMx 36 on package pin 7 */
#define ENCODERA_E1B_IIDX                                   (DL_GPIO_IIDX_DIO14)
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_14)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM36)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOA)

/* Defines for E2A: GPIOA.13 with pinCMx 35 on package pin 6 */
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO13)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_13)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM35)
/* Defines for E2B: GPIOA.12 with pinCMx 34 on package pin 5 */
#define ENCODERB_E2B_IIDX                                   (DL_GPIO_IIDX_DIO12)
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_12)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM34)
/* Port definition for Pin Group OLED_Pin */
#define OLED_Pin_PORT                                                    (GPIOA)

/* Defines for SDA: GPIOA.0 with pinCMx 1 on package pin 33 */
#define OLED_Pin_SDA_PIN                                         (DL_GPIO_PIN_0)
#define OLED_Pin_SDA_IOMUX                                        (IOMUX_PINCM1)
/* Defines for SCL: GPIOA.1 with pinCMx 2 on package pin 34 */
#define OLED_Pin_SCL_PIN                                         (DL_GPIO_PIN_1)
#define OLED_Pin_SCL_IOMUX                                        (IOMUX_PINCM2)
/* Port definition for Pin Group Key_Pins */
#define Key_Pins_PORT                                                    (GPIOB)

/* Defines for PIN_0: GPIOB.10 with pinCMx 27 on package pin 62 */
#define Key_Pins_PIN_0_PIN                                      (DL_GPIO_PIN_10)
#define Key_Pins_PIN_0_IOMUX                                     (IOMUX_PINCM27)
/* Defines for PIN_1: GPIOB.8 with pinCMx 25 on package pin 60 */
#define Key_Pins_PIN_1_PIN                                       (DL_GPIO_PIN_8)
#define Key_Pins_PIN_1_IOMUX                                     (IOMUX_PINCM25)
/* Defines for PIN_2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define Key_Pins_PIN_2_PIN                                       (DL_GPIO_PIN_7)
#define Key_Pins_PIN_2_IOMUX                                     (IOMUX_PINCM24)
/* Defines for PIN_3: GPIOB.6 with pinCMx 23 on package pin 58 */
#define Key_Pins_PIN_3_PIN                                       (DL_GPIO_PIN_6)
#define Key_Pins_PIN_3_IOMUX                                     (IOMUX_PINCM23)
/* Port definition for Pin Group Ultrasonic_Pins */
#define Ultrasonic_Pins_PORT                                             (GPIOA)

/* Defines for Trig_Pin: GPIOA.5 with pinCMx 10 on package pin 45 */
#define Ultrasonic_Pins_Trig_Pin_PIN                             (DL_GPIO_PIN_5)
#define Ultrasonic_Pins_Trig_Pin_IOMUX                           (IOMUX_PINCM10)
/* Defines for Echo_Pin: GPIOA.6 with pinCMx 11 on package pin 46 */
#define Ultrasonic_Pins_Echo_Pin_PIN                             (DL_GPIO_PIN_6)
#define Ultrasonic_Pins_Echo_Pin_IOMUX                           (IOMUX_PINCM11)

    /* clang-format on */

    void SYSCFG_DL_init(void);
    void SYSCFG_DL_initPower(void);
    void SYSCFG_DL_GPIO_init(void);
    void SYSCFG_DL_SYSCTL_init(void);
    void SYSCFG_DL_PWM_0_init(void);
    void SYSCFG_DL_CAPTURE_0_init(void);
    void SYSCFG_DL_TIMER_0_init(void);
    void SYSCFG_DL_TIMER_US_init(void);
    void SYSCFG_DL_UART_0_init(void);

    bool SYSCFG_DL_saveConfiguration(void);
    bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
