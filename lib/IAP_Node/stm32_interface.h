#pragma once

#ifndef __STM32_INTERFACE_H
#define __STM32_INTERFACE_H

#include <canard.h>
#include <canard/helpers.h>
#include <canard/interface.h>

typedef enum
{
  CAN_UNKNOWN = 0,
  CAN_125KBAUD = 1,
  CAN_250KBAUD = 2,
  CAN_500KBAUD = 3,
  CAN_1MBAUD = 4,
  CAN_UNDEFINED = 999,
} can_speed_t;

class STM32_CanardInterface : public Canard::Interface
{
  friend class UAVCAN_Node;

public:
  /// @brief Interface constructor
  /// @param _index index of the interface, used to identify which HandlerList to use
  /// @param _canfd true if the interface is CAN FD
  STM32_CanardInterface(uint8_t iface_index = 0) : Interface(iface_index) {}

  HAL_StatusTypeDef init(can_speed_t can_speed, uint32_t node_id);

  HAL_StatusTypeDef autobaud_init(volatile can_speed_t *can_speed, uint32_t node_id);

  /// @brief broadcast message
  /// @param bcast_transfer transfer to broadcast
  /// @return true if broadcast was added to the queue
  bool broadcast(const Canard::Transfer &bcast_transfer) override;

  /// @brief request message from
  /// @param destination_node_id
  /// @param req_transfer
  /// @return true if request was added to the queue
  bool request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) override;

  /// @brief respond to a request
  /// @param destination_node_id
  /// @param res_transfer
  /// @return true if response was added to the queue
  bool respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) override;

  /// @brief get the node ID of the interface
  /// @return returns anonymous node ID 0 if not implemented
  uint8_t get_node_id() const override {
    return STM32_CanardInterface::canard.node_id;
  }

  /// @brief set the node ID of the interface
  /// @param node_id
  void set_node_id(uint8_t node_id) { 
    canardSetLocalNodeID(&(STM32_CanardInterface::canard), node_id);
  }

  void process(uint32_t duration_ms);

  static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *rx_transfer);

  static bool shouldAcceptTransfer(const CanardInstance *ins,
                                   uint64_t *out_data_type_signature,
                                   uint16_t data_type_id,
                                   CanardTransferType transfer_type,
                                   uint8_t source_node_id);

private:
  uint8_t memory_pool[1024];
  CanardInstance canard;
  CanardTxTransfer tx_transfer;
  uint8_t unique_id[16];

  uint32_t _tick_overflow_count = 0;
  uint32_t _last_tick = 0;

  uint32_t getMicros32(void);
  uint64_t getMicros64(void);
  uint32_t getMillis32(void);
  void getUniqueID(uint8_t id[16]);
};

#ifdef __cplusplus
extern "C"
{
#endif
can_speed_t can_freq2speed(int freq);
uint32_t can_speed2freq(can_speed_t speed);
#ifdef __cplusplus
}
#endif

#endif /* __STM32_INTERFACE_H */