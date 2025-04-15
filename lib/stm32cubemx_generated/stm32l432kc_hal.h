#ifndef __STM32L432KC_HAL_H
#define __STM32L432KC_HAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <Arduino.h>
#include "stm32l4xx_ll_adc.h"

#define VBUS_ADC_PIN PA0
#define IOUTA_ADC_PIN PA1
#define IOUTB_ADC_PIN PA2
#define IOUTC_ADC_PIN PA3

#define N_FAULT_PIN PA4
#define EN_GATE_PIN PA5
#define ALLERT_PIN PA6

#define INL_A_PIN PA7
#define INH_A_PIN PA8
#define INL_B_PIN PB0
#define INH_B_PIN PA9
#define INL_C_PIN PB1
#define INH_C_PIN PA10

#define CAN_RX_PIN PA11
#define CAN_TX_PIN PA12

#define SWDIO_PIN PA13
#define SWCLK_PIN PA14

#define SPI_NSS_PIN PA15
#define SPI_SCK_PIN PB3
#define SPI_MISO_PIN PB4
#define SPI_MOSI_PIN PB5

#define USART1_TX_PIN PB6
#define USART1_RX_PIN PB7

// #define PIN_SERIAL1_TX              USART1_TX_PIN
// #define PIN_SERIAL1_RX              USART1_RX_PIN

/* Analog read resolution */
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096

  extern ADC_HandleTypeDef hadc1;
  extern CAN_HandleTypeDef hcan1;
  extern SPI_HandleTypeDef hspi1;
  extern TIM_HandleTypeDef htim1;
  extern UART_HandleTypeDef huart1;

  void MX_ADC1_Init(void);
  void ADC_MspInit(ADC_HandleTypeDef *adcHandle);
  void ADC_MspDeInit(ADC_HandleTypeDef *adcHandle);
  void MX_CAN1_Init(void);
  void CAN_Filter_Config(void);
  void CAN_NVIC_Config(void);
  void CAN_MspInit(void);
  void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle);
  void MX_GPIO_Init(void);
  void SystemClock_Config(void);
  void MX_SPI1_Init(void);
  void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle);
  void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle);
  void HAL_MspInit(void);
  void MX_TIM1_Init(void);
  void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *tim_pwmHandle);
  void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle);
  void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *tim_pwmHandle);
  void MX_USART1_UART_Init(void);
  void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle);
  void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle);

#ifdef __cplusplus
}
#endif

#endif /* __STM32L432KC_HAL_H */