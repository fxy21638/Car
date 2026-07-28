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
extern "C" {
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



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                        TIMG12
#define PWM_0_INST_IRQHandler                                  TIMG12_IRQHandler
#define PWM_0_INST_INT_IRQN                                    (TIMG12_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_14
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM36)
#define GPIO_PWM_0_C0_IOMUX_FUNC                    IOMUX_PINCM36_PF_TIMG12_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_14
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM31)
#define GPIO_PWM_0_C1_IOMUX_FUNC                    IOMUX_PINCM31_PF_TIMG12_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA0)
#define TIMER_0_INST_IRQHandler                                 TIMA0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (499U)
/* Defines for TIMER_US */
#define TIMER_US_INST                                                    (TIMG0)
#define TIMER_US_INST_IRQHandler                                TIMG0_IRQHandler
#define TIMER_US_INST_INT_IRQN                                  (TIMG0_INT_IRQn)
#define TIMER_US_INST_LOAD_VALUE                                         (1999U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)





/* Port definition for Pin Group LEDB */
#define LEDB_PORT                                                        (GPIOA)

/* Defines for LED: GPIOA.2 with pinCMx 7 on package pin 8 */
#define LEDB_LED_PIN                                             (DL_GPIO_PIN_2)
#define LEDB_LED_IOMUX                                            (IOMUX_PINCM7)
/* Defines for S0: GPIOB.17 with pinCMx 43 on package pin 36 */
#define TRACK_SENSOR_S0_PORT                                             (GPIOB)
#define TRACK_SENSOR_S0_PIN                                     (DL_GPIO_PIN_17)
#define TRACK_SENSOR_S0_IOMUX                                    (IOMUX_PINCM43)
/* Defines for S1: GPIOB.18 with pinCMx 44 on package pin 37 */
#define TRACK_SENSOR_S1_PORT                                             (GPIOB)
#define TRACK_SENSOR_S1_PIN                                     (DL_GPIO_PIN_18)
#define TRACK_SENSOR_S1_IOMUX                                    (IOMUX_PINCM44)
/* Defines for S2: GPIOB.20 with pinCMx 48 on package pin 41 */
#define TRACK_SENSOR_S2_PORT                                             (GPIOB)
#define TRACK_SENSOR_S2_PIN                                     (DL_GPIO_PIN_20)
#define TRACK_SENSOR_S2_IOMUX                                    (IOMUX_PINCM48)
/* Defines for S3: GPIOA.22 with pinCMx 47 on package pin 40 */
#define TRACK_SENSOR_S3_PORT                                             (GPIOA)
#define TRACK_SENSOR_S3_PIN                                     (DL_GPIO_PIN_22)
#define TRACK_SENSOR_S3_IOMUX                                    (IOMUX_PINCM47)
/* Defines for S4: GPIOA.24 with pinCMx 54 on package pin 44 */
#define TRACK_SENSOR_S4_PORT                                             (GPIOA)
#define TRACK_SENSOR_S4_PIN                                     (DL_GPIO_PIN_24)
#define TRACK_SENSOR_S4_IOMUX                                    (IOMUX_PINCM54)
/* Defines for S5: GPIOA.25 with pinCMx 55 on package pin 45 */
#define TRACK_SENSOR_S5_PORT                                             (GPIOA)
#define TRACK_SENSOR_S5_PIN                                     (DL_GPIO_PIN_25)
#define TRACK_SENSOR_S5_IOMUX                                    (IOMUX_PINCM55)
/* Defines for S6: GPIOA.26 with pinCMx 59 on package pin 46 */
#define TRACK_SENSOR_S6_PORT                                             (GPIOA)
#define TRACK_SENSOR_S6_PIN                                     (DL_GPIO_PIN_26)
#define TRACK_SENSOR_S6_IOMUX                                    (IOMUX_PINCM59)
/* Defines for S7: GPIOA.27 with pinCMx 60 on package pin 47 */
#define TRACK_SENSOR_S7_PORT                                             (GPIOA)
#define TRACK_SENSOR_S7_PIN                                     (DL_GPIO_PIN_27)
#define TRACK_SENSOR_S7_IOMUX                                    (IOMUX_PINCM60)
/* Port definition for Pin Group MOTOR */
#define MOTOR_PORT                                                       (GPIOB)

/* Defines for AIN1: GPIOB.6 with pinCMx 23 on package pin 20 */
#define MOTOR_AIN1_PIN                                           (DL_GPIO_PIN_6)
#define MOTOR_AIN1_IOMUX                                         (IOMUX_PINCM23)
/* Defines for AIN2: GPIOB.7 with pinCMx 24 on package pin 21 */
#define MOTOR_AIN2_PIN                                           (DL_GPIO_PIN_7)
#define MOTOR_AIN2_IOMUX                                         (IOMUX_PINCM24)
/* Defines for BIN1: GPIOB.9 with pinCMx 26 on package pin 23 */
#define MOTOR_BIN1_PIN                                           (DL_GPIO_PIN_9)
#define MOTOR_BIN1_IOMUX                                         (IOMUX_PINCM26)
/* Defines for BIN2: GPIOB.8 with pinCMx 25 on package pin 22 */
#define MOTOR_BIN2_PIN                                           (DL_GPIO_PIN_8)
#define MOTOR_BIN2_IOMUX                                         (IOMUX_PINCM25)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOB)

/* Defines for E1A: GPIOB.3 with pinCMx 16 on package pin 15 */
// groups represented: ["ENCODERB","ENCODERA"]
// pins affected: ["E2A","E2B","E1A","E1B"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODERA_E1A_IIDX                                    (DL_GPIO_IIDX_DIO3)
#define ENCODERA_E1A_PIN                                         (DL_GPIO_PIN_3)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM16)
/* Defines for E1B: GPIOB.2 with pinCMx 15 on package pin 14 */
#define ENCODERA_E1B_IIDX                                    (DL_GPIO_IIDX_DIO2)
#define ENCODERA_E1B_PIN                                         (DL_GPIO_PIN_2)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM15)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOB)

/* Defines for E2A: GPIOB.16 with pinCMx 33 on package pin 26 */
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO16)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_16)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM33)
/* Defines for E2B: GPIOB.15 with pinCMx 32 on package pin 25 */
#define ENCODERB_E2B_IIDX                                   (DL_GPIO_IIDX_DIO15)
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_15)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM32)
/* Port definition for Pin Group OLED_Pin */
#define OLED_Pin_PORT                                                    (GPIOA)

/* Defines for SDA: GPIOA.0 with pinCMx 1 on package pin 1 */
#define OLED_Pin_SDA_PIN                                         (DL_GPIO_PIN_0)
#define OLED_Pin_SDA_IOMUX                                        (IOMUX_PINCM1)
/* Defines for SCL: GPIOA.1 with pinCMx 2 on package pin 2 */
#define OLED_Pin_SCL_PIN                                         (DL_GPIO_PIN_1)
#define OLED_Pin_SCL_IOMUX                                        (IOMUX_PINCM2)
/* Port definition for Pin Group Key_Pins */
#define Key_Pins_PORT                                                    (GPIOA)

/* Defines for PIN_0: GPIOA.8 with pinCMx 19 on package pin 16 */
#define Key_Pins_PIN_0_PIN                                       (DL_GPIO_PIN_8)
#define Key_Pins_PIN_0_IOMUX                                     (IOMUX_PINCM19)
/* Defines for PIN_1: GPIOA.9 with pinCMx 20 on package pin 17 */
#define Key_Pins_PIN_1_PIN                                       (DL_GPIO_PIN_9)
#define Key_Pins_PIN_1_IOMUX                                     (IOMUX_PINCM20)
/* Defines for PIN_2: GPIOA.28 with pinCMx 3 on package pin 3 */
#define Key_Pins_PIN_2_PIN                                      (DL_GPIO_PIN_28)
#define Key_Pins_PIN_2_IOMUX                                      (IOMUX_PINCM3)
/* Defines for PIN_3: GPIOA.31 with pinCMx 6 on package pin 5 */
#define Key_Pins_PIN_3_PIN                                      (DL_GPIO_PIN_31)
#define Key_Pins_PIN_3_IOMUX                                      (IOMUX_PINCM6)
/* Port definition for Pin Group Ultrasonic_Pins */
#define Ultrasonic_Pins_PORT                                             (GPIOA)

/* Defines for Trig_Pin: GPIOA.12 with pinCMx 34 on package pin 27 */
#define Ultrasonic_Pins_Trig_Pin_PIN                            (DL_GPIO_PIN_12)
#define Ultrasonic_Pins_Trig_Pin_IOMUX                           (IOMUX_PINCM34)
/* Defines for Echo_Pin: GPIOA.13 with pinCMx 35 on package pin 28 */
#define Ultrasonic_Pins_Echo_Pin_PIN                            (DL_GPIO_PIN_13)
#define Ultrasonic_Pins_Echo_Pin_IOMUX                           (IOMUX_PINCM35)
/* Port definition for Pin Group MPU6050 */
#define MPU6050_PORT                                                     (GPIOA)

/* Defines for MPU_SDA: GPIOA.7 with pinCMx 14 on package pin 13 */
#define MPU6050_MPU_SDA_PIN                                      (DL_GPIO_PIN_7)
#define MPU6050_MPU_SDA_IOMUX                                    (IOMUX_PINCM14)
/* Defines for MPU_SCL: GPIOA.6 with pinCMx 11 on package pin 12 */
#define MPU6050_MPU_SCL_PIN                                      (DL_GPIO_PIN_6)
#define MPU6050_MPU_SCL_IOMUX                                    (IOMUX_PINCM11)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_TIMER_US_init(void);
void SYSCFG_DL_UART_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
