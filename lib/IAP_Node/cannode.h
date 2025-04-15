#pragma once

#ifndef __CANNODE_H
#define __CANNODE_H

#include <canard.h>
#include <canard/helpers.h>
#include <canard/interface.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <canard/handler_list.h>
#include <canard/transfer_object.h>
#include <dronecan_msgs.h>
#include <drivers/stm32/canard_stm32.h>

#include <stm32_interface.h>
// #include <bootloader.h>

#ifndef PREFERRED_NODE_ID
#define PREFERRED_NODE_ID 73
#endif
#ifndef NODE_NAME
#define NODE_NAME "BOOTLOADER"
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif

typedef enum
{
  STOP = 0,
  READY,
  START,
  END,
} update_status;

typedef struct
{
  char *name;
  enum uavcan_protocol_param_Value_type_t type;
  float value;
  float min_value;
  float max_value;
} Node_InfoTypeDef;

typedef struct
{
  uint32_t request_interval_ms;
  uint32_t uid_offset;
} Node_DNATypeDef;

typedef struct
{
  char path[256];
  uint8_t server_id;
  uint32_t last_read_ms;
  uint32_t offset;
  uint32_t address;
  uint32_t fw_image_size;
  update_status status;
} FW_UpdateTypeDef;

// DEFINE_HANDLER_LIST_HEADS();
// DEFINE_TRANSFER_OBJECT_HEADS();

#ifdef __cplusplus
extern "C"
{
#endif
  void handle_file_Read(const CanardRxTransfer &transfer, const uavcan_protocol_file_ReadResponse &res);
#ifdef __cplusplus
}
#endif

class UAVCAN_Node
{
public:
  STM32_CanardInterface stm32canard_iface{0};
  Node_DNATypeDef dna;
  volatile uint32_t recv_wait_ms = 2;
  HAL_StatusTypeDef allocate_node_id(void);
  uint8_t start_node(void);

private:
  // parameters
  Node_InfoTypeDef parameters[1] = {
      {(char *)"BOOTLOADER", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 0, 0, 127},
  };

  // publisher part
  // declare publishers for outgoing messages
  void send_NodeStatus(void);
  Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub{stm32canard_iface};
  void send_LogMessage(uavcan_protocol_debug_LogLevel level, uint8_t stage, uint8_t status);
  Canard::Publisher<uavcan_protocol_debug_LogMessage> log_message_pub{stm32canard_iface};
  void request_DNA(void);
  Canard::Publisher<uavcan_protocol_dynamic_node_id_Allocation> allocation_pub{stm32canard_iface};

  // subscriber part
  // handlers for dynamic node allocation (DNA)
  void handle_DNA_Allocation(const CanardRxTransfer &transfer, const uavcan_protocol_dynamic_node_id_Allocation &msg);
  Canard::ObjCallback<UAVCAN_Node, uavcan_protocol_dynamic_node_id_Allocation> allocation_cb{this, &UAVCAN_Node::handle_DNA_Allocation};
  Canard::Subscriber<uavcan_protocol_dynamic_node_id_Allocation> allocation_listener{allocation_cb, 0};

  // server part
  // Node Info
  void handle_GetNodeInfo(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest &req);
  Canard::ObjCallback<UAVCAN_Node, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{this, &UAVCAN_Node::handle_GetNodeInfo};
  Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{stm32canard_iface, node_info_req_cb};
  // Restart devce
  void handle_RestartNode(const CanardRxTransfer &transfer, const uavcan_protocol_RestartNodeRequest &req);
  Canard::ObjCallback<UAVCAN_Node, uavcan_protocol_RestartNodeRequest> node_restart_req_cb{this, &UAVCAN_Node::handle_RestartNode};
  Canard::Server<uavcan_protocol_RestartNodeRequest> node_restart_server{stm32canard_iface, node_restart_req_cb};
  // parameter getset
  void handle_param_GetSet(const CanardRxTransfer &transfer, const uavcan_protocol_param_GetSetRequest &req);
  Canard::ObjCallback<UAVCAN_Node, uavcan_protocol_param_GetSetRequest> param_get_set_req_cb{this, &UAVCAN_Node::handle_param_GetSet};
  Canard::Server<uavcan_protocol_param_GetSetRequest> param_server{stm32canard_iface, param_get_set_req_cb};
  // parameter execute opcode
  void handle_param_ExecuteOpcode(const CanardRxTransfer &transfer, const uavcan_protocol_param_ExecuteOpcodeRequest &req);
  Canard::ObjCallback<UAVCAN_Node, uavcan_protocol_param_ExecuteOpcodeRequest> param_executeopcode_req_cb{this, &UAVCAN_Node::handle_param_ExecuteOpcode};
  Canard::Server<uavcan_protocol_param_ExecuteOpcodeRequest> param_opcode_server{stm32canard_iface, param_executeopcode_req_cb};
  // firmware update
  void handle_begin_firmware_Update(const CanardRxTransfer &transfer, const uavcan_protocol_file_BeginFirmwareUpdateRequest &req);
  Canard::ObjCallback<UAVCAN_Node, uavcan_protocol_file_BeginFirmwareUpdateRequest> begin_fw_update_req_cb{this, &UAVCAN_Node::handle_begin_firmware_Update};
  Canard::Server<uavcan_protocol_file_BeginFirmwareUpdateRequest> fw_update_server{stm32canard_iface, begin_fw_update_req_cb};

  // client part
  void send_firmware_data_Request(void);
  void handle_file_read_response(const CanardRxTransfer &transfer, const uavcan_protocol_file_ReadResponse &res);
  Canard::ObjCallback<UAVCAN_Node, uavcan_protocol_file_ReadResponse> file_read_res_cb{this, &UAVCAN_Node::handle_file_read_response};
  Canard::Client<uavcan_protocol_file_ReadResponse> file_read_client{stm32canard_iface, file_read_res_cb};

  void send_firmware_info_Request(void);
  void handle_file_info_response(const CanardRxTransfer &transfer, const uavcan_protocol_file_GetInfoResponse &res);
  Canard::ObjCallback<UAVCAN_Node, uavcan_protocol_file_GetInfoResponse> file_info_res_cb{this, &UAVCAN_Node::handle_file_info_response};
  Canard::Client<uavcan_protocol_file_GetInfoResponse> file_info_client{stm32canard_iface, file_info_res_cb};
};

#endif /* __CANNODE_H */