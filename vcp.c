#include "vcp.h"
#include "usb_descriptors.h"
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#include <stdarg.h>

static char vcp_tx_buffer[200];
static uint8_t vcp_tx_in, vcp_tx_out;

static CDC_LineEncoding_t LineEncoding = { .BaudRateBPS = 0,
                                           .CharFormat  = CDC_LINEENCODING_OneStopBit,
                                           .ParityType  = CDC_PARITY_None,
                                           .DataBits    = 8                            };



/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the CDC management task started.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
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
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
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
			}

			break;
		case CDC_REQ_SetControlLineState:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();

				/* NOTE: Here you can read in the line state mask from the host, to get the current state of the output handshake
				         lines. The mask is read in from the wValue parameter in USB_ControlRequest, and can be masked against the
						 CONTROL_LINE_OUT_* masks to determine the RTS and DTR line states using the following code:
				*/
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

	if ((vcp_tx_in != vcp_tx_out) && LineEncoding.BaudRateBPS)
	{
            /* Select the Serial Tx Endpoint */
            Endpoint_SelectEndpoint(CDC_TX_EPADDR);
	    
	    uint16_t bytes_processed;
	    uint16_t to_write;
	    
            if(vcp_tx_out > vcp_tx_in) {
              // write everything until end
              to_write = sizeof(vcp_tx_buffer) - vcp_tx_out;
              
              bytes_processed = 0;
              if(Endpoint_Write_Stream_LE(&vcp_tx_buffer[vcp_tx_out], to_write, &bytes_processed) == ENDPOINT_RWSTREAM_IncompleteTransfer) {
                vcp_tx_out += bytes_processed;
              } else {
                vcp_tx_out += to_write;
              }

              if(vcp_tx_out >= sizeof(vcp_tx_buffer)) {
                vcp_tx_out = 0;
              }
            }

            if(vcp_tx_out < vcp_tx_in) {
              to_write = vcp_tx_in - vcp_tx_out;
              bytes_processed = 0;

              if(Endpoint_Write_Stream_LE(&vcp_tx_buffer[vcp_tx_out], to_write, &bytes_processed) == ENDPOINT_RWSTREAM_IncompleteTransfer) {
                vcp_tx_out += bytes_processed;
              } else {
                vcp_tx_out += to_write;
              }
              
              if(vcp_tx_out >= sizeof(vcp_tx_buffer)) {
                vcp_tx_out = 0;
              }
            }

            /* Remember if the packet to send completely fills the endpoint */
            bool IsFull = (Endpoint_BytesInEndpoint() == CDC_TXRX_EPSIZE);

            /* Finalize the stream transfer to send the last packet */
            Endpoint_ClearIN();

            /* If the last packet filled the endpoint, send an empty packet to release the buffer on
             * the receiver (otherwise all data will be cached until a non-full packet is received) */
            if (IsFull)
            {
                    /* Wait until the endpoint is ready for another packet */
                    Endpoint_WaitUntilReady();

                    /* Send an empty packet to ensure that the host does not buffer data sent to it */
                    Endpoint_ClearIN();
            }
	}

	/* Select the Serial Rx Endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	/* Throw away any received data from the host */
	if (Endpoint_IsOUTReceived())
	  Endpoint_ClearOUT();
}

void VCP_Printf(const char *fmt, ...)
{
  va_list ap;
  char tmp[256];
  
  va_start(ap, fmt);
  
  vsnprintf(tmp, sizeof(tmp) - 1, fmt, ap);
  
  tmp[sizeof(tmp) - 1] = 0;
  
  uint8_t l = strlen(tmp);

  for(uint8_t i = 0; i < l; ++i) {
    vcp_tx_buffer[vcp_tx_in++] = tmp[i];
    if(vcp_tx_in == sizeof(vcp_tx_buffer)) {
      vcp_tx_in = 0;
    }
  }
  
  va_end(ap);
}
