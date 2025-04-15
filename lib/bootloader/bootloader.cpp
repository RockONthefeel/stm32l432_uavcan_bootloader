#include <Arduino.h>
#include <CRC64.h>
#include <CRC32.h>
#include <flash.h>
#include <cannode.h>
#include "bootloader.h"

#ifdef __cplusplus
extern "C"
{
#endif

  BootloaderTypeDef bootloader;
  SharedTypeDef common;

  static void read_shared(SharedTypeDef *pshared)
  {
    pshared->signature = getreg32(signature_LOC);
    pshared->bus_speed = getreg32(bus_speed_LOC);
    pshared->node_id = getreg32(node_id_LOC);
    pshared->crc.ul[0] = getreg32(crc_LoLOC);
    pshared->crc.ul[1] = getreg32(crc_HiLOC);
  }

  static void write_shared(SharedTypeDef *pshared)
  {
    putreg32(pshared->signature, signature_LOC);
    putreg32(pshared->bus_speed, bus_speed_LOC);
    putreg32(pshared->node_id, node_id_LOC);
    putreg32(pshared->crc.ul[0], crc_LoLOC);
    putreg32(pshared->crc.ul[1], crc_HiLOC);
  }

  static uint64_t calulate_signature(SharedTypeDef *pshared)
  {
    CRC64 crc;
    crc.setInitial(0xFFFFFFFFFFFFFFFFull);
    crc.add(pshared->signature);
    crc.add(pshared->bus_speed);
    crc.add(pshared->node_id);
    return crc.calc() ^ 0xFFFFFFFFFFFFFFFFull;
  }

  static void bootloader_app_shared_init(SharedTypeDef *pshared, eRole_t role)
  {
    memset(pshared, 0, sizeof(SharedTypeDef));

    if (role != Invalid)
    {
      pshared->signature = (role == App ? BOOTLOADER_COMMON_APP_SIGNATURE : BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
    }
  }

  int bootloader_app_shared_read(SharedTypeDef *shared, eRole_t role)
  {
    int rv = HAL_ERROR;
    SharedTypeDef working;

    read_shared(&working);

    if ((role == App ? working.signature == BOOTLOADER_COMMON_APP_SIGNATURE
                     : working.signature == BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE) &&
        (working.crc.ull == calulate_signature(&working)))
    {
      *shared = working;
      rv = HAL_OK;
    }
    return rv;
  }

  void bootloader_app_shared_write(SharedTypeDef *shared, eRole_t role)
  {
    SharedTypeDef working = *shared;
    working.signature = (role == App ? BOOTLOADER_COMMON_APP_SIGNATURE : BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
    working.crc.ull = calulate_signature(&working);
    write_shared(&working);
  }

  void bootloader_app_shared_invalidate(void)
  {
    SharedTypeDef working;
    bootloader_app_shared_init(&working, Invalid);
    write_shared(&working);
  }

  static void find_descriptor(void)
  {
    uint8_t *p = (uint8_t *)APP_START_ADDRESS;
    APP_DescriptorTypeDef *descriptor = NULL;
    uint8_t signature[APP_DESCRIPTOR_SIGNATURE_SIZE] = APP_DESCRIPTOR_SIGNATURE;

    for (uint32_t offset = 0; offset < APP_DATA_MAX_SIZE - APP_DESCRIPTOR_SIGNATURE_SIZE; ++offset)
    {
      if (memcmp(p + offset, signature, APP_DESCRIPTOR_SIGNATURE_SIZE) == 0)
      {
        descriptor = (APP_DescriptorTypeDef *)(p + offset);
        break;
      }
    }
    bootloader.fw_image_descriptor = descriptor;
  }

  bool is_app_valid(volatile uint64_t *first_words)
  {
    CRC32 block_crc1;
    CRC32 block_crc2;
    uint32_t block1_crc;
    uint32_t block2_crc;
    size_t length;
    size_t block1_len;
    size_t block2_len;

    find_descriptor();

    if (!bootloader.fw_image_descriptor || first_words[0] == 0xFFFFFFFFu)
    {
      return false;
    }

    length = bootloader.fw_image_descriptor->image_size;

    if (length > APP_DATA_MAX_SIZE || length == 0)
    {
      return false;
    }

    block1_len = (size_t)&bootloader.fw_image_descriptor->crc.image_crc - (size_t)bootloader.fw_image;
    block2_len = length - ((size_t)&bootloader.fw_image_descriptor->major_version - (size_t)bootloader.fw_image);

    if (block2_len > APP_DATA_MAX_SIZE || block2_len == 0)
    {
      return false;
    }

    block_crc1.setInitial(0);
    for (uint32_t i = 0; i < 8; i++)
    {
      uint8_t *data = (uint8_t *)first_words + i;
      block_crc1.add(*data);
    }
    for (uint32_t i = 8; i < block1_len; i++)
    {
      uint8_t *data = (uint8_t *)APP_START_ADDRESS + i;
      block_crc1.add(*data);
    }
    // block_crc1.add((const uint8_t *)APP_START_ADDRESS, block1_len);
    // block_crc1.add((const uint8_t *)(bootloader.fw_image + 1), (size_t)(&bootloader.fw_image_descriptor->crc.crc32_block1) - (size_t)(bootloader.fw_image + 1));

    block_crc2.setInitial(0);
    block_crc2.add((const uint8_t *)&bootloader.fw_image_descriptor->major_version, block2_len);

    block1_crc = block_crc1.calc();
    block2_crc = block_crc2.calc();

    return block1_crc == bootloader.fw_image_descriptor->crc.crc32_block1 && block2_crc == bootloader.fw_image_descriptor->crc.crc32_block2;
  }

#ifdef __cplusplus
}
#endif

boot_status_t bootloader_init(UAVCAN_Node &node)
{
  char error_log_stage;

  __HAL_RCC_CAN1_CLK_ENABLE();

  error_log_stage = LOGMESSAGE_STAGE_INIT;

  bootloader.app_bl_request = false;
  bootloader.app_valid = false;
  bootloader.bus_speed = CAN_UNDEFINED;

  bootloader.firmware.address = APP_START_ADDRESS;
  bootloader.firmware.fw_image_size = 0;
  bootloader.firmware.last_read_ms = 0;
  bootloader.firmware.offset = 0;
  memset((uint8_t *)bootloader.firmware.path, 0, sizeof(bootloader.firmware.path));
  bootloader.firmware.server_id = 0;
  bootloader.firmware.status = STOP;

  bootloader.fw_image = (volatile uint64_t *)(APP_START_ADDRESS);
  bootloader.fw_image_descriptor = NULL;
  bootloader.fw_word0.l = 0;
  bootloader.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
  bootloader.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
  bootloader.percentage_done = 0;
  bootloader.sub_mode = 0;
  bootloader.uptime = 0;

  common.crc.valid = false;
  common.bus_speed = 0;
  common.node_id = 0;
  common.signature = 0;

  bootloader.app_valid = is_app_valid(bootloader.fw_image);
  if (!bootloader.app_valid)
  {
    if (HAL_OK != node.stm32canard_iface.autobaud_init(&bootloader.bus_speed, common.node_id))
    {
      return BOOT_FAILED;
    }
    common.bus_speed = can_speed2freq(bootloader.bus_speed);
    return BOOT_STAY;
  }

  if (HAL_OK == bootloader_app_shared_read(&common, App))
  {
    while(bootloader.app_bl_request != true)
    {
      bootloader.app_bl_request = true;
    }
  }

  bootloader_app_shared_invalidate();

  if (bootloader.app_bl_request)
  {
    while(bootloader.bus_speed != can_freq2speed(common.bus_speed))
    {
      bootloader.bus_speed = can_freq2speed(common.bus_speed);
    }
    
    if (bootloader.bus_speed != CAN_UNDEFINED)
    {
      if (HAL_OK != node.stm32canard_iface.init(bootloader.bus_speed, common.node_id))
      {
        return BOOT_FAILED;
      }
      common.crc.valid = true;
    }
    else
    {
      if (HAL_OK != node.stm32canard_iface.autobaud_init(&bootloader.bus_speed, common.node_id))
      {
        return BOOT_FAILED;
      }
      common.bus_speed = can_speed2freq(bootloader.bus_speed);
    }
    return BOOT_STAY;
  }
  else
  {
    return BOOT_APP;
  }
}