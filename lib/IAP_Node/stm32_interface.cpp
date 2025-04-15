#include <Arduino.h>
#include <stm32l432kc_hal.h>

#include <canard.h>
#include <dronecan_msgs.h>
#include <drivers/stm32/canard_stm32.h>

#include "stm32_interface.h"

#define UID_ADDR 0x1FFF7590
#define MY_NODE_ID 0

#ifdef __cplusplus
extern "C"
{
#endif

  can_speed_t can_freq2speed(int freq)
  {
    return (freq == 1000000u  ? CAN_1MBAUD
            : freq == 500000u ? CAN_500KBAUD
            : freq == 250000u ? CAN_250KBAUD
                              : CAN_125KBAUD);
  }

  uint32_t can_speed2freq(can_speed_t speed)
  {
    return 1000000 >> (CAN_1MBAUD - speed);
  }

#ifdef __cplusplus
}
#endif

uint32_t STM32_CanardInterface::getMicros32(void)
{
  return getCurrentMicros();
}

uint64_t STM32_CanardInterface::getMicros64(void)
{
  uint32_t current_tick = getCurrentMillis();
  if (current_tick < STM32_CanardInterface::_last_tick)
  {
    STM32_CanardInterface::_tick_overflow_count++;
  }
  STM32_CanardInterface::_last_tick = current_tick;

  __IO uint32_t u = SysTick->VAL;
  const uint32_t tms = SysTick->LOAD + 1;
  uint32_t micros = ((tms - u) * 1000) / tms;

  uint64_t total_micros = ((uint64_t)(STM32_CanardInterface::_tick_overflow_count) << 32) |
                          ((uint64_t)current_tick * 1000 + micros);

  return total_micros;
}

uint32_t STM32_CanardInterface::getMillis32(void)
{
  return getCurrentMillis();
}

void STM32_CanardInterface::getUniqueID(uint8_t id[16])
{
  memset(id, 0, 16);
  uint32_t *uid = (uint32_t *)UID_ADDR;
  for (int i = 0; i < 3; i++)
  {
    id[i * 4 + 0] = (uid[i] >> 24) & 0xFF;
    id[i * 4 + 1] = (uid[i] >> 16) & 0xFF;
    id[i * 4 + 2] = (uid[i] >> 8) & 0xFF;
    id[i * 4 + 3] = (uid[i]) & 0xFF;
  }
}

HAL_StatusTypeDef STM32_CanardInterface::init(can_speed_t can_speed, uint32_t node_id)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* CAN1 clock enable */
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**CAN1 GPIO Configuration
  PA11     ------> CAN1_RX
  PA12     ------> CAN1_TX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  CanardSTM32CANTimings timings;
  timings.bit_rate_prescaler = 16;
  timings.bit_segment_1 = 3;
  timings.bit_segment_2 = 1;
  timings.max_resynchronization_jump_width = 1;
  int16_t result = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
  if (result < 0)
  {
    return HAL_ERROR;
  }

  STM32_CanardInterface::getUniqueID(STM32_CanardInterface::unique_id);

  canardInit(&(STM32_CanardInterface::canard),
             STM32_CanardInterface::memory_pool,
             sizeof(STM32_CanardInterface::memory_pool),
             STM32_CanardInterface::onTransferReceived,
             STM32_CanardInterface::shouldAcceptTransfer,
             this);

  STM32_CanardInterface::set_node_id(node_id);
  return HAL_OK;
}

HAL_StatusTypeDef STM32_CanardInterface::autobaud_init(volatile can_speed_t *can_speed, uint32_t node_id)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* CAN1 clock enable */
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**CAN1 GPIO Configuration
  PA11     ------> CAN1_RX
  PA12     ------> CAN1_TX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  CanardSTM32CANTimings timings;
  timings.bit_rate_prescaler = 16;
  timings.bit_segment_1 = 3;
  timings.bit_segment_2 = 1;
  timings.max_resynchronization_jump_width = 1;
  int16_t result = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
  if (result < 0)
  {
    return HAL_ERROR;
  }

  STM32_CanardInterface::getUniqueID(STM32_CanardInterface::unique_id);

  canardInit(&(STM32_CanardInterface::canard),
             STM32_CanardInterface::memory_pool,
             sizeof(STM32_CanardInterface::memory_pool),
             STM32_CanardInterface::onTransferReceived,
             STM32_CanardInterface::shouldAcceptTransfer,
             this);

  *can_speed = CAN_1MBAUD;
  STM32_CanardInterface::set_node_id(node_id);
  return HAL_OK;
}

bool STM32_CanardInterface::broadcast(const Canard::Transfer &bcast_transfer)
{
  STM32_CanardInterface::tx_transfer = {
      .transfer_type = bcast_transfer.transfer_type,
      .data_type_signature = bcast_transfer.data_type_signature,
      .data_type_id = bcast_transfer.data_type_id,
      .inout_transfer_id = bcast_transfer.inout_transfer_id,
      .priority = bcast_transfer.priority,
      .payload = (const uint8_t *)bcast_transfer.payload,
      .payload_len = uint16_t(bcast_transfer.payload_len),
#if CANARD_ENABLE_CANFD
      .canfd = bcast_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
      .deadline_usec = STM32_CanardInterface::getMicros64() + (bcast_transfer.timeout_ms * 1000),
#endif
#if CANARD_MULTI_IFACE
      .iface_mask = uint8_t((1 << num_ifaces) - 1),
#endif
  };
  // do canard broadcast
  return canardBroadcastObj(&(STM32_CanardInterface::canard), &(STM32_CanardInterface::tx_transfer)) > 0;
}

bool STM32_CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer)
{
  STM32_CanardInterface::tx_transfer = {
      .transfer_type = req_transfer.transfer_type,
      .data_type_signature = req_transfer.data_type_signature,
      .data_type_id = req_transfer.data_type_id,
      .inout_transfer_id = req_transfer.inout_transfer_id,
      .priority = req_transfer.priority,
      .payload = (const uint8_t *)req_transfer.payload,
      .payload_len = uint16_t(req_transfer.payload_len),
#if CANARD_ENABLE_CANFD
      .canfd = req_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
      .deadline_usec = STM32_CanardInterface::getMicros64() + (req_transfer.timeout_ms * 1000),
#endif
#if CANARD_MULTI_IFACE
      .iface_mask = uint8_t((1 << num_ifaces) - 1),
#endif
  };
  // do canard request
  return canardRequestOrRespondObj(&(STM32_CanardInterface::canard), destination_node_id, &(STM32_CanardInterface::tx_transfer)) > 0;
}

bool STM32_CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer)
{
  STM32_CanardInterface::tx_transfer = {
      .transfer_type = res_transfer.transfer_type,
      .data_type_signature = res_transfer.data_type_signature,
      .data_type_id = res_transfer.data_type_id,
      .inout_transfer_id = res_transfer.inout_transfer_id,
      .priority = res_transfer.priority,
      .payload = (const uint8_t *)res_transfer.payload,
      .payload_len = uint16_t(res_transfer.payload_len),
#if CANARD_ENABLE_CANFD
      .canfd = res_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
      .deadline_usec = STM32_CanardInterface::getMicros64() + (res_transfer.timeout_ms * 1000),
#endif
#if CANARD_MULTI_IFACE
      .iface_mask = uint8_t((1 << num_ifaces) - 1),
#endif
  };
  // do canard respond
  return canardRequestOrRespondObj(&(STM32_CanardInterface::canard), destination_node_id, &(STM32_CanardInterface::tx_transfer)) > 0;
}

void STM32_CanardInterface::process(uint32_t timeout_msec)
{
  CanardCANFrame rx_frame;
  
  // Transmitting
  for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&(STM32_CanardInterface::canard))) != NULL;)
  {
    const int16_t tx_res = canardSTM32Transmit(txf);
    if (tx_res != 0)
    {
      canardPopTxQueue(&(STM32_CanardInterface::canard));
    }
  }

  // Receiving
  const uint32_t start_ms = STM32_CanardInterface::getMillis32();
  while (STM32_CanardInterface::getMillis32() - start_ms < timeout_msec)
  {
    const int16_t rx_res = canardSTM32Receive(&rx_frame);
    if (rx_res > 0)
    {
      canardHandleRxFrame(&(STM32_CanardInterface::canard), &rx_frame, STM32_CanardInterface::getMicros64());
    }
    else
    {
      yield();
    }
  }
}

void STM32_CanardInterface::onTransferReceived(CanardInstance *ins, CanardRxTransfer *rx_transfer)
{
  STM32_CanardInterface *iface = (STM32_CanardInterface *)ins->user_reference;
  iface->handle_message(*rx_transfer);
}

bool STM32_CanardInterface::shouldAcceptTransfer(const CanardInstance *ins,
                                                 uint64_t *out_data_type_signature,
                                                 uint16_t data_type_id,
                                                 CanardTransferType transfer_type,
                                                 uint8_t source_node_id)
{
  STM32_CanardInterface *iface = (STM32_CanardInterface *)ins->user_reference;
  return iface->accept_message(data_type_id, transfer_type, *out_data_type_signature);
}