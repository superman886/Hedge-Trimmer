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



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM */
#define PWM_INST                                                           TIMG6
#define PWM_INST_IRQHandler                                     TIMG6_IRQHandler
#define PWM_INST_INT_IRQN                                       (TIMG6_INT_IRQn)
#define PWM_INST_CLK_FREQ                                                4000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_C0_PORT                                                   GPIOA
#define GPIO_PWM_C0_PIN                                           DL_GPIO_PIN_21
#define GPIO_PWM_C0_IOMUX                                        (IOMUX_PINCM46)
#define GPIO_PWM_C0_IOMUX_FUNC                       IOMUX_PINCM46_PF_TIMG6_CCP0
#define GPIO_PWM_C0_IDX                                      DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_C1_PORT                                                   GPIOB
#define GPIO_PWM_C1_PIN                                            DL_GPIO_PIN_7
#define GPIO_PWM_C1_IOMUX                                        (IOMUX_PINCM24)
#define GPIO_PWM_C1_IOMUX_FUNC                       IOMUX_PINCM24_PF_TIMG6_CCP1
#define GPIO_PWM_C1_IDX                                      DL_TIMER_CC_1_INDEX



/* Defines for TIMER_TICK */
#define TIMER_TICK_INST                                                  (TIMA0)
#define TIMER_TICK_INST_IRQHandler                              TIMA0_IRQHandler
#define TIMER_TICK_INST_INT_IRQN                                (TIMA0_INT_IRQn)
#define TIMER_TICK_INST_LOAD_VALUE                                      (63999U)



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
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_4_MHZ_9600_BAUD                                         (26)
#define UART_0_FBRD_4_MHZ_9600_BAUD                                          (3)





/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for LED1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define LED_LED1_PIN                                            (DL_GPIO_PIN_14)
#define LED_LED1_IOMUX                                           (IOMUX_PINCM31)
/* Port definition for Pin Group Buzzer */
#define Buzzer_PORT                                                      (GPIOB)

/* Defines for Buzzer1: GPIOB.13 with pinCMx 30 on package pin 1 */
#define Buzzer_Buzzer1_PIN                                      (DL_GPIO_PIN_13)
#define Buzzer_Buzzer1_IOMUX                                     (IOMUX_PINCM30)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for KEY1: GPIOB.9 with pinCMx 26 on package pin 61 */
// pins affected by this interrupt request:["KEY1","KEY2","KEY3"]
#define KEY_INT_IRQN                                            (GPIOB_INT_IRQn)
#define KEY_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define KEY_KEY1_IIDX                                        (DL_GPIO_IIDX_DIO9)
#define KEY_KEY1_PIN                                             (DL_GPIO_PIN_9)
#define KEY_KEY1_IOMUX                                           (IOMUX_PINCM26)
/* Defines for KEY2: GPIOB.2 with pinCMx 15 on package pin 50 */
#define KEY_KEY2_IIDX                                        (DL_GPIO_IIDX_DIO2)
#define KEY_KEY2_PIN                                             (DL_GPIO_PIN_2)
#define KEY_KEY2_IOMUX                                           (IOMUX_PINCM15)
/* Defines for KEY3: GPIOB.18 with pinCMx 44 on package pin 15 */
#define KEY_KEY3_IIDX                                       (DL_GPIO_IIDX_DIO18)
#define KEY_KEY3_PIN                                            (DL_GPIO_PIN_18)
#define KEY_KEY3_IOMUX                                           (IOMUX_PINCM44)
/* Port definition for Pin Group TB6612 */
#define TB6612_PORT                                                      (GPIOA)

/* Defines for AIN1: GPIOA.8 with pinCMx 19 on package pin 54 */
#define TB6612_AIN1_PIN                                          (DL_GPIO_PIN_8)
#define TB6612_AIN1_IOMUX                                        (IOMUX_PINCM19)
/* Defines for AIN2: GPIOA.9 with pinCMx 20 on package pin 55 */
#define TB6612_AIN2_PIN                                          (DL_GPIO_PIN_9)
#define TB6612_AIN2_IOMUX                                        (IOMUX_PINCM20)
/* Defines for BIN2: GPIOA.31 with pinCMx 6 on package pin 39 */
#define TB6612_BIN2_PIN                                         (DL_GPIO_PIN_31)
#define TB6612_BIN2_IOMUX                                         (IOMUX_PINCM6)
/* Defines for BIN1: GPIOA.28 with pinCMx 3 on package pin 35 */
#define TB6612_BIN1_PIN                                         (DL_GPIO_PIN_28)
#define TB6612_BIN1_IOMUX                                         (IOMUX_PINCM3)
/* Port definition for Pin Group GPIO */
#define GPIO_PORT                                                        (GPIOA)

/* Defines for SDA: GPIOA.1 with pinCMx 2 on package pin 34 */
#define GPIO_SDA_PIN                                             (DL_GPIO_PIN_1)
#define GPIO_SDA_IOMUX                                            (IOMUX_PINCM2)
/* Defines for SCL: GPIOA.0 with pinCMx 1 on package pin 33 */
#define GPIO_SCL_PIN                                             (DL_GPIO_PIN_0)
#define GPIO_SCL_IOMUX                                            (IOMUX_PINCM1)
/* Port definition for Pin Group IRtracking */
#define IRtracking_PORT                                                  (GPIOA)

/* Defines for DO3: GPIOA.25 with pinCMx 55 on package pin 26 */
#define IRtracking_DO3_PIN                                      (DL_GPIO_PIN_25)
#define IRtracking_DO3_IOMUX                                     (IOMUX_PINCM55)
/* Defines for DO1: GPIOA.27 with pinCMx 60 on package pin 31 */
#define IRtracking_DO1_PIN                                      (DL_GPIO_PIN_27)
#define IRtracking_DO1_IOMUX                                     (IOMUX_PINCM60)
/* Defines for DO2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define IRtracking_DO2_PIN                                      (DL_GPIO_PIN_26)
#define IRtracking_DO2_IOMUX                                     (IOMUX_PINCM59)
/* Defines for DO4: GPIOA.24 with pinCMx 54 on package pin 25 */
#define IRtracking_DO4_PIN                                      (DL_GPIO_PIN_24)
#define IRtracking_DO4_IOMUX                                     (IOMUX_PINCM54)
/* Defines for DO5: GPIOA.23 with pinCMx 53 on package pin 24 */
#define IRtracking_DO5_PIN                                      (DL_GPIO_PIN_23)
#define IRtracking_DO5_IOMUX                                     (IOMUX_PINCM53)
/* Defines for DO6: GPIOA.22 with pinCMx 47 on package pin 18 */
#define IRtracking_DO6_PIN                                      (DL_GPIO_PIN_22)
#define IRtracking_DO6_IOMUX                                     (IOMUX_PINCM47)
/* Defines for DO7: GPIOA.15 with pinCMx 37 on package pin 8 */
#define IRtracking_DO7_PIN                                      (DL_GPIO_PIN_15)
#define IRtracking_DO7_IOMUX                                     (IOMUX_PINCM37)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOA)

/* Defines for E1A: GPIOA.12 with pinCMx 34 on package pin 5 */
// groups represented: ["ENCODERB","ENCODERA"]
// pins affected: ["E2A","E1A","E1B"]
#define GPIO_MULTIPLE_GPIOA_INT_IRQN                            (GPIOA_INT_IRQn)
#define GPIO_MULTIPLE_GPIOA_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO12)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_12)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM34)
/* Defines for E1B: GPIOA.13 with pinCMx 35 on package pin 6 */
#define ENCODERA_E1B_IIDX                                   (DL_GPIO_IIDX_DIO13)
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_13)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM35)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOA)

/* Defines for E2A: GPIOA.17 with pinCMx 39 on package pin 10 */
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO17)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_17)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM39)
/* Defines for E2B: GPIOA.16 with pinCMx 38 on package pin 9 */
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_16)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM38)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_init(void);
void SYSCFG_DL_TIMER_TICK_init(void);
void SYSCFG_DL_UART_0_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
