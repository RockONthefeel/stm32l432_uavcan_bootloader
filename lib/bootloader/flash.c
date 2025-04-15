#include <Arduino.h>
// #include <bootloader.h>
#include <stdio.h>
#include "flash.h"

#define APP_START_ADDRESS 0x08010000

void jump_to_app(void)
{
  typedef void (*pFunction)(void);
  pFunction jump_to_application;

  uint32_t app_start_address = *(__IO uint32_t *)(APP_START_ADDRESS + 4);
  jump_to_application = (pFunction)app_start_address;

  __set_MSP(*(__IO uint32_t *)APP_START_ADDRESS);

  jump_to_application();
}

HAL_StatusTypeDef bl_flash_erase_page(uint32_t address, uint32_t num_pages)
{
  if (address < APP_START_ADDRESS || address > FLASH_END)
  {
    return HAL_ERROR;
  }

  uint32_t page_start_address = address & ~(FLASH_PAGE_SIZE - 1);
  uint32_t page_index = (page_start_address - FLASH_BASE) / FLASH_PAGE_SIZE;

  if (page_start_address + num_pages * FLASH_PAGE_SIZE - 1 > FLASH_END)
  {
    return HAL_ERROR;
  }

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase_init;
  uint32_t page_error;
  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.Banks = FLASH_BANK_1;

  for (uint32_t i = 0; i < num_pages; i++)
  {
    bool need_erase = false;
    uint32_t current_page_addr = page_start_address + i * FLASH_PAGE_SIZE;

    for (uint32_t j = 0; j < FLASH_PAGE_SIZE; j += 4)
    {
      if (*(volatile uint32_t *)(current_page_addr + j) != 0xFFFFFFFF)
      {
        need_erase = true;
        break;
      }
    }

    if (need_erase)
    {
      erase_init.Page = page_index + i;
      erase_init.NbPages = 1;
      HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
      if (status != HAL_OK)
      {
        HAL_FLASH_Lock();
        printf("Erase failed at page: 0x%08lx, error: %d\n",
               page_error * FLASH_PAGE_SIZE + FLASH_BASE, status);
        return status;
      }
    }
  }

  HAL_FLASH_Lock();
  return HAL_OK;
}

HAL_StatusTypeDef bl_flash_write_fw(uint32_t address, uint8_t *data, uint32_t size)
{
  if (address < APP_START_ADDRESS || address > FLASH_END)
  {
    return HAL_ERROR;
  }

  if (address + size - 1 > FLASH_END)
  {
    return HAL_ERROR;
  }

  HAL_FLASH_Unlock();

  for (uint32_t i = 0; i < size; i += 8)
  {
    uint64_t doubleword_data = 0;

    for (uint32_t j = 0; j < 8; j++)
    {
      if (i + j < size)
      {
        doubleword_data |= ((uint64_t)data[i + j] << (8 * j));
      }
    }
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, doubleword_data);
    if (status != HAL_OK)
    {
      HAL_FLASH_Lock();
      printf("Flash write status: %d\n", status);
      return status;
    }
  }

  HAL_FLASH_Lock();
  return HAL_OK;
}