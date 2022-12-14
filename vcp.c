#include "vcp.h"
#include "usb_descriptors.h"
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>
#include "fifo.h"

#include <stdarg.h>

#define VCP_RX_FIFO_SIZE 64
#define VCP_TX_FIFO_SIZE 64

static struct {
	fifo_t fifo;
	uint8_t _alloc[VCP_RX_FIFO_SIZE];
} rx = {
	.fifo     = {
		.in   = 0,
		.out  = 0,
		.size = VCP_RX_FIFO_SIZE,
	}
};

static struct {
	fifo_t fifo;
	uint8_t _alloc[VCP_TX_FIFO_SIZE];
} tx = {
	.fifo     = {
		.in   = 0,
		.out  = 0,
		.size = VCP_TX_FIFO_SIZE,
	}
};

static CDC_LineEncoding_t LineEncoding = { .BaudRateBPS = 0,
	                                   .CharFormat  = CDC_LINEENCODING_OneStopBit,
	                                   .ParityType  = CDC_PARITY_None,
	                                   .DataBits    = 8 };


static uint16_t usb_write_sync(const void *buf, uint16_t len);


static int usb_putchar(char c, FILE *stream)
{
	if (fdev_get_udata(stream) && (c == '\n')) {
		usb_putchar('\r', stream);
	}

	tx.fifo.buffer[tx.fifo.in++] = c;

	if (tx.fifo.in == tx.fifo.size) {
		usb_write_sync(tx.fifo.buffer, tx.fifo.in);
		tx.fifo.in = 0;
	}
	return 0;
}

static int usb_getchar(FILE *stream)
{
	uint8_t buf;

	if (fifo_read(&rx.fifo, &buf, 1) == 1) {
		return buf;
	} else {
		return _FDEV_ERR;
	}
}

FILE *VCP_Init(void)
{
	static FILE myfp = FDEV_SETUP_STREAM(usb_putchar, usb_getchar, _FDEV_SETUP_RW);

	return &myfp;
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
	ConfigSuccess &= Endpoint_ConfigureEndpoint(CDC_RX_EPADDR, EP_TYPE_BULK, CDC_TXRX_EPSIZE, 1);

	/* Reset line encoding baud rate so that the host knows to send new values */
	LineEncoding.BaudRateBPS = 0;

	/* Indicate endpoint configuration success or failure */
// LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);

	return ConfigSuccess;
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void VCP_EVENT_USB_Device_ControlRequest(void)
{
	/* Process CDC specific control requests */
	switch (USB_ControlRequest.bRequest) {
	case CDC_REQ_GetLineEncoding:
		if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE)) {
			Endpoint_ClearSETUP();

			/* Write the line coding data to the control endpoint */
			Endpoint_Write_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
			Endpoint_ClearOUT();
		}

		break;
	case CDC_REQ_SetLineEncoding:
		if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE)) {
			Endpoint_ClearSETUP();

			/* Read the line coding data in from the host into the global struct */
			Endpoint_Read_Control_Stream_LE(&LineEncoding, sizeof(CDC_LineEncoding_t));
			Endpoint_ClearIN();

// EVENT_VCP_SetLineEncoding(&LineEncoding);
		}

		break;
	case CDC_REQ_SetControlLineState:
		if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE)) {
			uint16_t wValue = USB_ControlRequest.wValue;
			Endpoint_ClearSETUP();
			Endpoint_ClearStatusStage();

			/* NOTE: Here you can read in the line state mask from the host, to get the current state of the output handshake
			         lines. The mask is read in from the wValue parameter in USB_ControlRequest, and can be masked against the
			                 CONTROL_LINE_OUT_* masks to determine the RTS and DTR line states using the following code:
			 */
// EVENT_VCP_SetControlLineState(wValue);
		}

		break;
	}
}

/** Function to manage CDC data transmission and reception to and from the host. */
void VCP_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured) {
		return;
	}

	if (tx.fifo.in > 0) {
		usb_write_sync(tx.fifo.buffer, tx.fifo.in);
		tx.fifo.in = 0;
	}

	/* Select the Serial Rx Endpoint */
	Endpoint_SelectEndpoint(CDC_RX_EPADDR);

	if (Endpoint_IsOUTReceived()) {
		uint16_t DataLength = Endpoint_BytesInEndpoint();

		if (DataLength <= fifo_avail_write(&rx.fifo)) {
			uint8_t Buffer[DataLength];

			Endpoint_Read_Stream_LE(&Buffer, DataLength, NULL);

			fifo_write(&rx.fifo, Buffer, DataLength);

			Endpoint_ClearOUT();

// EVENT_VCP_DataReceived();
		}
	}
}

static uint16_t usb_write_sync(const void *buf, uint16_t len)
{
	if (USB_DeviceState != DEVICE_STATE_Configured) {
		return 0;
	}

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

	if (needZLP) {
		Endpoint_WaitUntilReady();
		Endpoint_ClearIN();
	}

	return bytes_processed;
}
