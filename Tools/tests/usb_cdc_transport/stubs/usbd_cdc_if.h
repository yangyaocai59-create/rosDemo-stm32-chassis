#ifndef TEST_USBD_CDC_IF_H
#define TEST_USBD_CDC_IF_H
#include <stdint.h>
typedef struct { uint32_t GINTMSK; } TestUsbRegisters;
extern TestUsbRegisters test_usb_registers;
#define USB_OTG_FS (&test_usb_registers)
#define USB_OTG_GINTMSK_SOFM (1U << 3)
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define __DMB() ((void)0)
uint32_t HAL_GetTick(void);
uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);
#endif
