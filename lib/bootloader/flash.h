#pragma once

#ifndef __FLASH_H
#define __FLASH_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C"
{
#endif

  void jump_to_app(void);

  HAL_StatusTypeDef bl_flash_erase_page(uint32_t page_address, uint32_t num_pages);
  HAL_StatusTypeDef bl_flash_write_fw(uint32_t address, uint8_t *data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_H */