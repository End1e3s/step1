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
#define CONFIG_MSPM0G3507

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



/* Defines for PWM_WHEEL */
#define PWM_WHEEL_INST                                                     TIMA1
#define PWM_WHEEL_INST_IRQHandler                               TIMA1_IRQHandler
#define PWM_WHEEL_INST_INT_IRQN                                 (TIMA1_INT_IRQn)
#define PWM_WHEEL_INST_CLK_FREQ                                         80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_WHEEL_C0_PORT                                             GPIOA
#define GPIO_PWM_WHEEL_C0_PIN                                     DL_GPIO_PIN_17
#define GPIO_PWM_WHEEL_C0_IOMUX                                  (IOMUX_PINCM39)
#define GPIO_PWM_WHEEL_C0_IOMUX_FUNC                 IOMUX_PINCM39_PF_TIMA1_CCP0
#define GPIO_PWM_WHEEL_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_WHEEL_C1_PORT                                             GPIOA
#define GPIO_PWM_WHEEL_C1_PIN                                     DL_GPIO_PIN_11
#define GPIO_PWM_WHEEL_C1_IOMUX                                  (IOMUX_PINCM22)
#define GPIO_PWM_WHEEL_C1_IOMUX_FUNC                 IOMUX_PINCM22_PF_TIMA1_CCP1
#define GPIO_PWM_WHEEL_C1_IDX                                DL_TIMER_CC_1_INDEX




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOB
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOB
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL



/* Port definition for Pin Group GPIO_LED */
#define GPIO_LED_PORT                                                    (GPIOB)

/* Defines for PIN_0: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_LED_PIN_0_PIN                                      (DL_GPIO_PIN_22)
#define GPIO_LED_PIN_0_IOMUX                                     (IOMUX_PINCM50)
/* Port definition for Pin Group GPIO_SENSOR */
#define GPIO_SENSOR_PORT                                                 (GPIOB)

/* Defines for SENSOR_LEFTMOST_PIN0: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_SENSOR_SENSOR_LEFTMOST_PIN0_PIN                     (DL_GPIO_PIN_8)
#define GPIO_SENSOR_SENSOR_LEFTMOST_PIN0_IOMUX                   (IOMUX_PINCM25)
/* Defines for SENSOR_LEFT_PIN0: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_SENSOR_SENSOR_LEFT_PIN0_PIN                         (DL_GPIO_PIN_7)
#define GPIO_SENSOR_SENSOR_LEFT_PIN0_IOMUX                       (IOMUX_PINCM24)
/* Defines for SENSOR_CENTER_PIN0: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_SENSOR_SENSOR_CENTER_PIN0_PIN                       (DL_GPIO_PIN_6)
#define GPIO_SENSOR_SENSOR_CENTER_PIN0_IOMUX                     (IOMUX_PINCM23)
/* Defines for SENSOR_RIGHT_PIN0: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_SENSOR_SENSOR_RIGHT_PIN0_PIN                        (DL_GPIO_PIN_0)
#define GPIO_SENSOR_SENSOR_RIGHT_PIN0_IOMUX                      (IOMUX_PINCM12)
/* Defines for SENSOR_RIGHTMOST_PIN0: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_SENSOR_SENSOR_RIGHTMOST_PIN0_PIN                   (DL_GPIO_PIN_16)
#define GPIO_SENSOR_SENSOR_RIGHTMOST_PIN0_IOMUX                  (IOMUX_PINCM33)
/* Defines for PIN_A1: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GPIO_ENCODER_L_PIN_A1_PORT                                       (GPIOA)
#define GPIO_ENCODER_L_PIN_A1_PIN                               (DL_GPIO_PIN_26)
#define GPIO_ENCODER_L_PIN_A1_IOMUX                              (IOMUX_PINCM59)
/* Defines for PIN_B1: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_ENCODER_L_PIN_B1_PORT                                       (GPIOB)
#define GPIO_ENCODER_L_PIN_B1_PIN                               (DL_GPIO_PIN_24)
#define GPIO_ENCODER_L_PIN_B1_IOMUX                              (IOMUX_PINCM52)
/* Defines for PIN_A2: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_ENCODER_R_PIN_A2_PORT                                       (GPIOB)
#define GPIO_ENCODER_R_PIN_A2_PIN                                (DL_GPIO_PIN_9)
#define GPIO_ENCODER_R_PIN_A2_IOMUX                              (IOMUX_PINCM26)
/* Defines for PIN_B2: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_ENCODER_R_PIN_B2_PORT                                       (GPIOA)
#define GPIO_ENCODER_R_PIN_B2_PIN                               (DL_GPIO_PIN_27)
#define GPIO_ENCODER_R_PIN_B2_IOMUX                              (IOMUX_PINCM60)
/* Defines for AIN_2: GPIOA.28 with pinCMx 3 on package pin 35 */
#define GPIO_6612_AIN_2_PORT                                             (GPIOA)
#define GPIO_6612_AIN_2_PIN                                     (DL_GPIO_PIN_28)
#define GPIO_6612_AIN_2_IOMUX                                     (IOMUX_PINCM3)
/* Defines for AIN_1: GPIOA.31 with pinCMx 6 on package pin 39 */
#define GPIO_6612_AIN_1_PORT                                             (GPIOA)
#define GPIO_6612_AIN_1_PIN                                     (DL_GPIO_PIN_31)
#define GPIO_6612_AIN_1_IOMUX                                     (IOMUX_PINCM6)
/* Defines for STBY: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_6612_STBY_PORT                                              (GPIOB)
#define GPIO_6612_STBY_PIN                                      (DL_GPIO_PIN_12)
#define GPIO_6612_STBY_IOMUX                                     (IOMUX_PINCM29)
/* Defines for BIN_1: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_6612_BIN_1_PORT                                             (GPIOA)
#define GPIO_6612_BIN_1_PIN                                     (DL_GPIO_PIN_12)
#define GPIO_6612_BIN_1_IOMUX                                    (IOMUX_PINCM34)
/* Defines for BIN_2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_6612_BIN_2_PORT                                             (GPIOA)
#define GPIO_6612_BIN_2_PIN                                     (DL_GPIO_PIN_13)
#define GPIO_6612_BIN_2_IOMUX                                    (IOMUX_PINCM35)
/* Defines for PIN_1: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_BUTTON_PIN_1_PORT                                           (GPIOB)
#define GPIO_BUTTON_PIN_1_PIN                                   (DL_GPIO_PIN_21)
#define GPIO_BUTTON_PIN_1_IOMUX                                  (IOMUX_PINCM49)
/* Defines for PIN_2: GPIOA.18 with pinCMx 40 on package pin 11 */
#define GPIO_BUTTON_PIN_2_PORT                                           (GPIOA)
#define GPIO_BUTTON_PIN_2_PIN                                   (DL_GPIO_PIN_18)
#define GPIO_BUTTON_PIN_2_IOMUX                                  (IOMUX_PINCM40)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_WHEEL_init(void);
void SYSCFG_DL_I2C_OLED_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
