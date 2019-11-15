#include "vcp.h"
#include "usb_descriptors.h"
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>
#include "led.h"

#include <stdarg.h>

#define VCP_RX_FIFO_SIZE 64

typedef struct {
  uint16_t in, out;
  uint16_t size;
  uint8_t buffer[0];
} vcp_fifo_t;

static struct {
  vcp_fifo_t fifo;
  uint8_t _alloc[VCP_RX_FIFO_SIZE];
} rx = { 
  .fifo = {
    .in = 0,
    .out = 0,
    .size = VCP_RX_FIFO_SIZE,
  }
};


static CDC_LineEncoding_t LineEncoding = { .BaudRateBPS = 0,
                                           .CharFormat  = CDC_LINEENCODING_OneStopBit,
                                           .ParityType  = CDC_PARITY_None,
                                           .DataBits    = 8                            };


static int fifo_write(vcp_fifo_t *fifo, const uint8_t *buf, int len)
{
  int i;
  for(i = 0; i < len; ++i) {
    fifo->buffer[fifo->in++] = buf[i];
    if(fifo->in == fifo->size) {
      fifo->in = 0;
    }

    if(fifo->in == fifo->out) { // overflow
      led_a = blink_pulse;
    }
  }
  return i;
}

static int fifo_read(vcp_fifo_t *fifo, uint8_t *buf, int len)
{
  int i;
  
  for(i = 0; (i < len) && (fifo->in != fifo->out); ++i) {
    buf[i] = fifo->buffer[fifo->out++];
    if(fifo->out == fifo->size) {
      fifo->out = 0;
    }
  }
  
  return i;
}

void VCP_Init(void)
{
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the CDC management task started.
 */
bool VCP_EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup CDC Data Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_NOTIFICATION_EPADDR, EP_TYPE_INTERRUPT, CDC_NOTIFICATION_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_TX_EPADDR, EP_TYPE_BULK, CDC_TXRX_EPSIZE, 1);
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_RX_EPADDR, EP_TYPE_BULK,  CDC_TXRX_EPSIZE, 1);

	/* Reset line encoding baud rate so that the host knows to send new values */
	LineEncoding.BaudRateBPS = 0;

	/* Indicate endpoint configuration success or failure */
//	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);

    return ConfigSuccess;
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void VCP_EVENT_USB_Device_ControlRequest(void)
{
	/* Process CDC specific control requests */
	switch (USB_ControlRequest.bRequest)
	{
		case CDC_REQ_GetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Write the line coding data to the control endpoint */
				Endpoint_Write_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearOUT();
			}

			break;
		case CDC_REQ_SetLineEncoding:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				/* Read the line coding data in from the host into the global struct */
				Endpoint_Read_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
				Endpoint_ClearIN();
                
                EVENT_VCP_SetLineEncoding(&LineEncoding);
			}

			break;
		case CDC_REQ_SetControlLineState:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
			        uint16_t wValue = USB_ControlRequest.wValue;
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				/* NOTE: Here you can read in the line state mask from the host, to get the current state of the output handshake
				         lines. The mask is read in from the wValue parameter in USB_ControlRequest, and can be masked against the
						 CONTROL_LINE_OUT_* masks to determine the RTS and DTR line states using the following code:
				*/
				EVENT_VCP_SetControlLineState(wValue);
			}

			break;
	}
}

/** Function to manage CDC data transmission and reception to and from the host. */
void VCP_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	/* Select the Serial Rx Endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	if (Endpoint_IsOUTReceived()) {
	  
	  uint16_t DataLength = Endpoint_BytesInEndpoint();
	  
	  uint8_t Buffer[DataLength];

	  Endpoint_Read_Stream_LE(&Buffer, DataLength, NULL);

	  cli();
          fifo_write(&rx.fifo, Buffer, DataLength);
          sei();

	  Endpoint_ClearOUT();
	  
	  EVENT_VCP_DataReceived();
        }
}

static uint16_t usb_write_sync(const void *buf, uint16_t len)
{
  if (USB_DeviceState != DEVICE_STATE_Configured)
    return 0;
  
  Endpoint_SelectEndpoint(CDC_TX_EPADDR);
  
  uint16_t bytes_processed = 0;
  uint8_t e;
  bool needZLP = false;

  do {
    Endpoint_WaitUntilReady();

    e = Endpoint_Write_Stream_LE(buf, len, &bytes_processed);
    
    needZLP = (Endpoint_BytesInEndpoint() == CDC_TXRX_EPSIZE);

    Endpoint_ClearIN();

  } while(e == ENDPOINT_RWSTREAM_IncompleteTransfer);

  if(needZLP) {
    Endpoint_WaitUntilReady();
    Endpoint_ClearIN();
  }
  
  return bytes_processed;
}

int VCP_Write(uint8_t *buf, int len)
{
  return usb_write_sync(buf, len);
}

int VCP_Read(uint8_t *buf, int len)
{
  return fifo_read(&rx.fifo, buf, len);
}

void VCP_Puts(const char *str)
{
  usb_write_sync(str, strlen(str));
}
