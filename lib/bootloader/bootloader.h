#pragma once

#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include <Arduino.h>
#include <stm32_interface.h>
#include <cannode.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LOGMESSAGE_STAGE_INIT 'I'
#define LOGMESSAGE_STAGE_GET_INFO 'G'
#define LOGMESSAGE_STAGE_ERASE 'E'
#define LOGMESSAGE_STAGE_READ 'R'
#define LOGMESSAGE_STAGE_PROGRAM 'P'
#define LOGMESSAGE_STAGE_VALIDATE 'V'
#define LOGMESSAGE_STAGE_FINALIZE 'F'

#define LOGMESSAGE_RESULT_START 's'
#define LOGMESSAGE_RESULT_FAIL 'f'
#define LOGMESSAGE_RESULT_OK 'o'

#define APP_START_ADDRESS 0x08010000
#define APP_DATA_MAX_SIZE (FLASH_END - APP_START_ADDRESS + 1)
#define APP_DESCRIPTOR_SIGNATURE {0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06}
#define APP_DESCRIPTOR_SIGNATURE_SIZE 8
#define BOOTLOADER_COMMON_APP_SIGNATURE 0xB0A04150
#define BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE 0xB0A0424C

#define getreg32(a) (*(volatile uint32_t *)(a))
#define putreg32(v, a) (*(volatile uint32_t *)(a) = (v))

#define STM32_CAN1_BASE CAN1_BASE
#define STM32_CAN_FIR_OFFSET(f, i) (0x240 + ((f) << 3) + (((i) - 1) << 2))

#define STM32_CAN1_FIR(b, i) (STM32_CAN1_BASE + STM32_CAN_FIR_OFFSET(b, i))

#define crc_HiLOC STM32_CAN1_FIR(2, 1)
#define crc_LoLOC STM32_CAN1_FIR(2, 2)
#define signature_LOC STM32_CAN1_FIR(3, 1)
#define bus_speed_LOC STM32_CAN1_FIR(3, 2)
#define node_id_LOC STM32_CAN1_FIR(4, 1)

typedef struct UAVCAN_Node UAVCAN_Node; 

  typedef enum
  {
    FLASH_OK = 0,
    FLASH_ERROR,
    FLASH_ERASE_ERROR,
    FLASH_ERASE_VERIFY_ERROR,
    FLASH_ERROR_SUICIDE,
    FLASH_ERROR_AFU,
  } flash_error_t;

  typedef enum
  {
    BOOT_APP = 0,
    BOOT_STAY,
    BOOT_FAILED,
  } boot_status_t;

  typedef enum
  {
    Invalid,
    App,
    BootLoader
  } eRole_t;

  typedef struct
  {
    uint8_t signature[sizeof(uint64_t)];
    union
    {
      uint64_t image_crc;
      struct
      {
        uint32_t crc32_block1;
        uint32_t crc32_block2;
      };
    } crc;
    uint32_t image_size;
    uint32_t git_hash;
    uint8_t major_version;
    uint8_t minor_version;
    uint16_t board_id;
    uint8_t reserved[3 + 3 + 2];
  } APP_DescriptorTypeDef;

  typedef volatile struct
  {
    can_speed_t bus_speed;
    volatile uint8_t health;
    volatile uint8_t mode;
    volatile uint8_t sub_mode;
    volatile bool app_valid;
    volatile uint32_t uptime;
    volatile APP_DescriptorTypeDef *fw_image_descriptor;
    FW_UpdateTypeDef firmware;
    volatile uint64_t *fw_image;
    bool app_bl_request;
    uint16_t percentage_done;
    union
    {
      uint64_t l;
      uint8_t b[sizeof(uint64_t)];
    } fw_word0;
  } BootloaderTypeDef;

  typedef struct
  {
    union
    {
      uint64_t ull;
      uint32_t ul[2];
      uint8_t valid;
    } crc;
    uint32_t signature;
    uint32_t bus_speed;
    uint32_t node_id;
  } SharedTypeDef;

  typedef struct uavcan_Path_t
  {
    uint8_t u8[200];
  } uavcan_Path_t;

  extern BootloaderTypeDef bootloader;
  extern SharedTypeDef common;

  int bootloader_app_shared_read(SharedTypeDef *shared, eRole_t role);
  void bootloader_app_shared_write(SharedTypeDef *shared, eRole_t role);
  void bootloader_app_shared_invalidate(void);
  bool is_app_valid(volatile uint64_t *first_words);
  boot_status_t bootloader_init(UAVCAN_Node &node);
  
#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H */