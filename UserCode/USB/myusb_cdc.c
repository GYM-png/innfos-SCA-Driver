#include "myusb_cdc.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define USB_OUT_BUFFER_SIZE 100

#if (USB_OUT_BUFFER_SIZE > 65535)
#error "usb_out_buffer_size"
#endif

static char usb_out_buffer[USB_OUT_BUFFER_SIZE];
static uint16_t output_length = 0;

int usb_cdc_println(const char * format, ...)
{
    memset(usb_out_buffer, 0, USB_OUT_BUFFER_SIZE);
    va_list args;
    va_start(args, format);
    output_length = vsnprintf(usb_out_buffer, USB_OUT_BUFFER_SIZE, format, args);
    output_length += sprintf(usb_out_buffer + output_length, "\r\n");
    va_end(args);

    return CDC_Transmit_HS(usb_out_buffer, output_length);
}
