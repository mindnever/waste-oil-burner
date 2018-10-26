#include "usb.h"
#ifdef USE_USB_VCP
# include "vcp.h"
#endif
#ifdef USE_USB_HID
# include "hid.h"
#endif

#include <LUFA/Drivers/USB/USB.h>

void USB_Task()
{
  USB_USBTask();
#ifdef USE_USB_VCP
  VCP_Task();
#endif
#ifdef USE_USB_HID
  HID_Task();
#endif
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the CDC management task started.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;
#ifdef USE_USB_VCP
    ConfigSuccess &= VCP_EVENT_USB_Device_ConfigurationChanged();
#endif
#ifdef USE_USB_HID
    ConfigSuccess &= HID_EVENT_USB_Device_ConfigurationChanged();
#endif
    USB_Device_EnableSOFEvents();
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
#ifdef USE_USB_VCP
    VCP_EVENT_USB_Device_ControlRequest();
#endif
#ifdef USE_USB_HID
    HID_EVENT_USB_Device_ControlRequest();
#endif
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
#ifdef USE_USB_HID
  HID_EVENT_USB_Device_StartOfFrame();
#endif
}

