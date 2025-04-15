#include <Arduino.h>
#include <IWatchdog.h>
#include <cannode.h>
#include <flash.h>
#include <bootloader.h>

DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS();

HAL_StatusTypeDef UAVCAN_Node::allocate_node_id(void)
{
  if (UAVCAN_Node::stm32canard_iface.get_node_id() == CANARD_BROADCAST_NODE_ID)
  {
    const uint32_t start_ms = stm32canard_iface.getMillis32();
    while ((stm32canard_iface.getMillis32() - start_ms < 5000) && (stm32canard_iface.get_node_id() == CANARD_BROADCAST_NODE_ID))
    {
      stm32canard_iface.process(5);
      if (stm32canard_iface.getMillis32() > UAVCAN_Node::dna.request_interval_ms)
      {
        UAVCAN_Node::request_DNA();
      }
    }

    if (UAVCAN_Node::stm32canard_iface.get_node_id() == CANARD_BROADCAST_NODE_ID)
    {
      return HAL_TIMEOUT;
    }
  }
  bootloader.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
  return HAL_OK;
}

uint8_t UAVCAN_Node::start_node(void)
{
  uint32_t next_1hz_service_at = stm32canard_iface.getMillis32();

  while (true)
  {
    stm32canard_iface.process(UAVCAN_Node::recv_wait_ms);

    switch (bootloader.firmware.status)
    {
    case STOP:
      break;

    case READY:
      UAVCAN_Node::send_firmware_info_Request();
      break;

    case START:
      UAVCAN_Node::send_firmware_data_Request();
      break;

    case END:
      if (!is_app_valid(&bootloader.fw_word0.l))
      {
        bootloader.app_valid = false;
        return BOOT_FAILED;
      }
      if (HAL_OK != bl_flash_write_fw(APP_START_ADDRESS, (uint8_t *)bootloader.fw_word0.b, sizeof(bootloader.fw_word0.b)))
      {
        bootloader.app_valid = false;
        return BOOT_FAILED;
      }
      return BOOT_APP;

    default:
      break;
    }

    const uint32_t ts = stm32canard_iface.getMillis32();
    if (ts >= next_1hz_service_at)
    {
      next_1hz_service_at += 1000ULL;
      UAVCAN_Node::send_NodeStatus();
      IWatchdog.reload();
    }
  }
}

void UAVCAN_Node::send_LogMessage(uavcan_protocol_debug_LogLevel level, uint8_t stage, uint8_t status)
{
  uavcan_protocol_debug_LogMessage msg{};

  msg.level.value = level.value;
  msg.text.data[0] = stage;
  msg.text.data[1] = status;

  log_message_pub.broadcast(msg);
}

void UAVCAN_Node::request_DNA(void)
{
  dna.request_interval_ms =
      stm32canard_iface.getMillis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
      (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

  // send allocation message
  uavcan_protocol_dynamic_node_id_Allocation msg{};

  msg.node_id = PREFERRED_NODE_ID;
  msg.first_part_of_unique_id = (dna.uid_offset == 0);

  uint8_t uid_size = (uint8_t)(16 - dna.uid_offset);

  if (uid_size > 6)
  {
    uid_size = 6;
  }

  msg.unique_id.len = uid_size;
  memcpy(msg.unique_id.data, &stm32canard_iface.unique_id[dna.uid_offset], uid_size);

  // Preparing for timeout; if response is received, this value will be updated from the callback.
  dna.uid_offset = 0;

  allocation_pub.broadcast(msg);
}

void UAVCAN_Node::handle_DNA_Allocation(const CanardRxTransfer &transfer, const uavcan_protocol_dynamic_node_id_Allocation &msg)
{
  if (stm32canard_iface.get_node_id() != CANARD_BROADCAST_NODE_ID)
  {
    return;
  }

  dna.request_interval_ms =
      stm32canard_iface.getMillis32() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
      (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

  if (transfer.source_node_id == CANARD_BROADCAST_NODE_ID)
  {
    Serial1.printf("Allocation request from another allocatee\n");
    dna.uid_offset = 0;
    return;
  }

  // Matching the received UID against the local one
  if (memcmp(msg.unique_id.data, stm32canard_iface.unique_id, msg.unique_id.len) != 0)
  {
    Serial1.printf("Mismatching allocation response\n");
    dna.uid_offset = 0;
    return;
  }

  if (msg.unique_id.len < sizeof(msg.unique_id.data))
  {
    // The allocator has confirmed part of unique ID, switching to
    // the next stage and updating the timeout.
    dna.uid_offset = msg.unique_id.len;
    dna.request_interval_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
  }
  else
  {
    // Allocation complete - copying the allocated node ID from the message
    stm32canard_iface.set_node_id(msg.node_id);
    Serial1.println("Matching allocation response:");
    Serial1.print("My unique ID: ");
    for (uint8_t i = 0; i < msg.unique_id.len; i++)
    {
      Serial1.printf("%02X", stm32canard_iface.unique_id[i]);
    }
    Serial1.printf("\nNode ID allocated: %d\n", msg.node_id);
  }
}

void UAVCAN_Node::send_NodeStatus(void)
{
  uavcan_protocol_NodeStatus msg{};

  msg.health = bootloader.health;
  msg.mode = bootloader.mode;
  msg.sub_mode = bootloader.sub_mode;
  msg.uptime_sec = stm32canard_iface.getMillis32() / 1000UL;
  if (bootloader.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE)
  {
    msg.vendor_specific_status_code = bootloader.percentage_done;
  }
  else
  {
    msg.vendor_specific_status_code = 0;
  }

  node_status_pub.broadcast(msg);
}

void UAVCAN_Node::handle_GetNodeInfo(const CanardRxTransfer &transfer, const uavcan_protocol_GetNodeInfoRequest &req)
{
  UNUSED(req);

  uavcan_protocol_GetNodeInfoResponse node_info_rsp;

  node_info_rsp.status.uptime_sec = stm32canard_iface.getMillis32() / 1000UL;
  node_info_rsp.status.sub_mode = bootloader.sub_mode;
  node_info_rsp.status.mode = bootloader.mode;
  node_info_rsp.status.health = bootloader.health;
  if (bootloader.mode == UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE)
  {
    node_info_rsp.status.vendor_specific_status_code = bootloader.percentage_done;
  }
  else
  {
    node_info_rsp.status.vendor_specific_status_code = 0;
  }

  node_info_rsp.hardware_version.major = 1;
  node_info_rsp.hardware_version.minor = 0;
  memcpy(node_info_rsp.hardware_version.unique_id, stm32canard_iface.unique_id, sizeof(UAVCAN_Node::stm32canard_iface.unique_id));
  strncpy((char *)node_info_rsp.name.data, NODE_NAME, sizeof(node_info_rsp.name.data));
  node_info_rsp.name.len = strnlen((char *)node_info_rsp.name.data, sizeof(node_info_rsp.name.data));

  memset(&node_info_rsp.software_version, 0, sizeof(node_info_rsp.software_version));
  if (bootloader.app_valid)
  {
    node_info_rsp.software_version.major = bootloader.fw_image_descriptor->major_version;
    node_info_rsp.software_version.minor = bootloader.fw_image_descriptor->minor_version;
    node_info_rsp.software_version.vcs_commit = bootloader.fw_image_descriptor->git_hash;
    node_info_rsp.software_version.image_crc = bootloader.fw_image_descriptor->crc.image_crc;
    node_info_rsp.software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC | UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT;
  }

  node_info_server.respond(transfer, node_info_rsp);
}

void UAVCAN_Node::handle_begin_firmware_Update(const CanardRxTransfer &transfer, const uavcan_protocol_file_BeginFirmwareUpdateRequest &req)
{
  // has already received the same begin firmware update request
  if (bootloader.firmware.server_id == transfer.source_node_id && memcmp((const char *)bootloader.firmware.path, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len) == 0)
  {
    return;
  }
  bootloader.firmware.address = APP_START_ADDRESS;
  bootloader.firmware.fw_image_size = 0;
  bootloader.firmware.last_read_ms = 0;
  bootloader.firmware.offset = 0;
  strncpy((char *)bootloader.firmware.path, (char *)req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);
  bootloader.firmware.server_id = transfer.source_node_id;
  bootloader.firmware.status = READY;

  bootloader.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;

  uavcan_protocol_file_BeginFirmwareUpdateResponse pkt{};
  pkt.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;
  fw_update_server.respond(transfer, pkt);
}

void UAVCAN_Node::send_firmware_info_Request(void)
{
  struct uavcan_protocol_file_GetInfoRequest pkt{};

  pkt.path.path.len = strlen((const char *)bootloader.firmware.path);
  memcpy(pkt.path.path.data, (const char *)bootloader.firmware.path, pkt.path.path.len);

  file_info_client.request(bootloader.firmware.server_id, pkt);

  // set a long wait time
  UAVCAN_Node::recv_wait_ms = 20;
}

void UAVCAN_Node::handle_file_info_response(const CanardRxTransfer &transfer, const uavcan_protocol_file_GetInfoResponse &res)
{
  uavcan_protocol_debug_LogLevel log_level;

  log_level.value = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO;

  if ((res.error.value == UAVCAN_PROTOCOL_FILE_ERROR_OK) &&
      (res.entry_type.flags & (UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_FILE | UAVCAN_PROTOCOL_FILE_ENTRYTYPE_FLAG_READABLE)) &&
      (res.size > 0) &&
      (res.size < APP_DATA_MAX_SIZE))
  {
    bootloader.firmware.fw_image_size = res.size;
    uint8_t num_pages = 0;
    if (bootloader.firmware.fw_image_size % FLASH_PAGE_SIZE != 0)
    {
      num_pages = bootloader.firmware.fw_image_size / FLASH_PAGE_SIZE + 1;
    }
    else
    {
      num_pages = bootloader.firmware.fw_image_size / FLASH_PAGE_SIZE;
    }

    if (HAL_OK != bl_flash_erase_page(APP_START_ADDRESS, num_pages))
    {
      bootloader.firmware.status = STOP;
      send_LogMessage(log_level, LOGMESSAGE_STAGE_ERASE, LOGMESSAGE_RESULT_FAIL);
    }
    else
    {
      bootloader.app_valid = false;
      bootloader.firmware.status = START;
      send_LogMessage(log_level, LOGMESSAGE_STAGE_ERASE, LOGMESSAGE_RESULT_OK);
    }
  }
  else
  {
    bootloader.firmware.status = STOP;
    send_LogMessage(log_level, LOGMESSAGE_STAGE_GET_INFO, LOGMESSAGE_RESULT_FAIL);
  }
  // set back a short wait time
  UAVCAN_Node::recv_wait_ms = 2;
}

void UAVCAN_Node::send_firmware_data_Request(void)
{
  uint32_t now = stm32canard_iface.getMillis32();
  if (now - bootloader.firmware.last_read_ms < 750)
  {
    return;
  }
  bootloader.firmware.last_read_ms = now;

  struct uavcan_protocol_file_ReadRequest pkt{};

  pkt.path.path.len = strlen((const char *)bootloader.firmware.path);
  pkt.offset = bootloader.firmware.offset;
  memcpy(pkt.path.path.data, (char *)bootloader.firmware.path, pkt.path.path.len);

  file_read_client.request(bootloader.firmware.server_id, pkt);
}

void UAVCAN_Node::handle_file_read_response(const CanardRxTransfer &transfer, const uavcan_protocol_file_ReadResponse &res)
{
  uavcan_protocol_debug_LogLevel log_level;
  log_level.value = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO;

  if (res.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK)
  {
    bootloader.firmware.address = APP_START_ADDRESS;
    bootloader.firmware.fw_image_size = 0;
    bootloader.firmware.last_read_ms = 0;
    bootloader.firmware.offset = 0;
    memset((char *)bootloader.firmware.path, 0, sizeof(bootloader.firmware.path));
    bootloader.firmware.server_id = 0;
    bootloader.firmware.status = STOP;

    send_LogMessage(log_level, LOGMESSAGE_STAGE_READ, LOGMESSAGE_RESULT_FAIL);
    return;
  }

  if (bootloader.firmware.offset + res.data.len > APP_DATA_MAX_SIZE)
  {
    bootloader.firmware.address = APP_START_ADDRESS;
    bootloader.firmware.fw_image_size = 0;
    bootloader.firmware.last_read_ms = 0;
    bootloader.firmware.offset = 0;
    memset((char *)bootloader.firmware.path, 0, sizeof(bootloader.firmware.path));
    bootloader.firmware.server_id = 0;
    bootloader.firmware.status = STOP;

    send_LogMessage(log_level, LOGMESSAGE_STAGE_READ, LOGMESSAGE_RESULT_FAIL);
    return;
  }

  if (bootloader.firmware.offset == 0)
  {
    memcpy((uint8_t *)bootloader.fw_word0.b, res.data.data, sizeof(bootloader.fw_word0.b));
    if (HAL_OK != bl_flash_write_fw(APP_START_ADDRESS + 8, (uint8_t *)res.data.data + 8, res.data.len - 8))
    {
      bootloader.firmware.address = APP_START_ADDRESS;
      bootloader.firmware.fw_image_size = 0;
      bootloader.firmware.last_read_ms = 0;
      bootloader.firmware.offset = 0;
      memset((char *)bootloader.firmware.path, 0, sizeof(bootloader.firmware.path));
      bootloader.firmware.server_id = 0;
      bootloader.firmware.status = STOP;

      send_LogMessage(log_level, LOGMESSAGE_STAGE_PROGRAM, LOGMESSAGE_RESULT_FAIL);
      return;
    }
    memset((uint8_t *)res.data.data, 0xFF, sizeof(bootloader.fw_word0.b));
  }
  else
  {
    if (HAL_OK != bl_flash_write_fw(bootloader.firmware.address, (uint8_t *)res.data.data, res.data.len))
    {
      bootloader.firmware.address = APP_START_ADDRESS;
      bootloader.firmware.fw_image_size = 0;
      bootloader.firmware.last_read_ms = 0;
      bootloader.firmware.offset = 0;
      memset((char *)bootloader.firmware.path, 0, sizeof(bootloader.firmware.path));
      bootloader.firmware.server_id = 0;
      bootloader.firmware.status = STOP;

      send_LogMessage(log_level, LOGMESSAGE_STAGE_PROGRAM, LOGMESSAGE_RESULT_FAIL);
      return;
    }
  }

  bootloader.firmware.address += res.data.len;
  bootloader.firmware.offset += res.data.len;
  bootloader.firmware.last_read_ms = 0;
  bootloader.percentage_done = 100 * bootloader.firmware.offset / bootloader.firmware.fw_image_size;

  if (res.data.len < 256)
  {
    bootloader.firmware.address = APP_START_ADDRESS;
    bootloader.firmware.fw_image_size = 0;
    bootloader.firmware.last_read_ms = 0;
    bootloader.firmware.offset = 0;
    memset((char *)bootloader.firmware.path, 0, sizeof(bootloader.firmware.path));
    bootloader.firmware.server_id = 0;
    bootloader.firmware.status = END;

    bootloader.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

    send_LogMessage(log_level, LOGMESSAGE_STAGE_VALIDATE, LOGMESSAGE_RESULT_OK);
    return;
  }
}

void UAVCAN_Node::handle_RestartNode(const CanardRxTransfer &transfer, const uavcan_protocol_RestartNodeRequest &req)
{
  Serial1.print("Get magic number: ");
  uint8_t magic_number[8];
  for (uint8_t i = 0; i < 8; i++)
  {
    magic_number[i] = (req.magic_number >> (i * 8)) & 0xFF;
    Serial1.printf("%02X ", magic_number[i]);
  }
  Serial1.println();
  Serial1.flush(100);

  uavcan_protocol_RestartNodeResponse pkt{};
  pkt.ok = true;
  node_restart_server.respond(transfer, pkt);

  stm32canard_iface.process(10);

  NVIC_SystemReset();
}

void UAVCAN_Node::handle_param_GetSet(const CanardRxTransfer &transfer, const uavcan_protocol_param_GetSetRequest &req)
{
  Node_InfoTypeDef *p = nullptr;
  if (req.name.len != 0)
  {
    for (uint16_t i = 0; i < ARRAY_SIZE(parameters); i++)
    {
      if (req.name.len == strlen(parameters[i].name) &&
          strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0)
      {
        p = &parameters[i];
        break;
      }
    }
  }
  else if (req.index < ARRAY_SIZE(parameters))
  {
    p = &parameters[req.index];
  }
  if (p != nullptr && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY)
  {
    switch (p->type)
    {
    case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
      p->value = req.value.integer_value;
      break;
    case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
      p->value = req.value.real_value;
      break;
    default:
      return;
    }
  }

  uavcan_protocol_param_GetSetResponse pkt{};

  if (p != NULL)
  {
    pkt.value.union_tag = p->type;
    switch (p->type)
    {
    case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
      pkt.value.integer_value = p->value;
      break;
    case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
      pkt.value.real_value = p->value;
      break;
    default:
      return;
    }
    pkt.name.len = strlen(p->name);
    strcpy((char *)pkt.name.data, p->name);
  }

  param_server.respond(transfer, pkt);
}

void UAVCAN_Node::handle_param_ExecuteOpcode(const CanardRxTransfer &transfer, const uavcan_protocol_param_ExecuteOpcodeRequest &req)
{
  if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE)
  {
    Serial1.printf("parameters erased!\n");
  }
  if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE)
  {
    Serial1.printf("parameters saved!\n");
  }

  uavcan_protocol_param_ExecuteOpcodeResponse pkt{};
  pkt.ok = true;
  param_opcode_server.respond(transfer, pkt);
}

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif
