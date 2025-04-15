#include <Arduino.h>
#include <IWatchdog.h>
#include <bootloader.h>
#include <cannode.h>
#include <stm32l432kc_hal.h>
#include <flash.h>
#include <CRC64.h>
#include <CRC32.h>

HardwareSerial Serial1(USART1);
UAVCAN_Node node;

void setup()
{
  if(IWatchdog.isReset())
  {
    IWatchdog.clearReset();
  }
  if(IWatchdog.isEnabled())
  {
    IWatchdog.set(30000000);
  }
  else
  {
    IWatchdog.begin(30000000);
  }
  
  Serial1.begin(115200);
  boot_status_t boot_status = bootloader_init(node);

  switch (boot_status)
  {
  case BOOT_APP:
    goto boot_app;
  case BOOT_STAY:
    goto boot_stay;
  case BOOT_FAILED:
    goto boot_error;
  }

boot_app:
{
  IWatchdog.reload();
  if (common.crc.valid)
  {
    bootloader_app_shared_write(&common, BootLoader);
  }
  jump_to_app();
}

boot_stay:
{
  if (node.allocate_node_id() != HAL_OK)
  {
    goto boot_error;
  }
  boot_status_t boot_status = (boot_status_t)node.start_node();
  switch (boot_status)
  {
  case BOOT_APP:
    goto boot_app;
  case BOOT_STAY:
    goto boot_stay;
  case BOOT_FAILED:
    goto boot_error;
  }
}

boot_error:
{
  delay(5000);
  NVIC_SystemReset();
}
}

void loop()
{
}

extern "C"
{
  int _write(int file, char *ptr, int len)
  {
    (void)file;
    Serial1.write(ptr, len);
    return len;
  }
}
