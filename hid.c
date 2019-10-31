#include <LUFA/Drivers/USB/Class/Device/HIDClassDevice.h>
#include "hid.h"
#include "usb_descriptors.h"
#include "HidSensorSpec.h"
#include "vcp.h"

#define REPORT_QUEUE_SIZE 8

struct ReportItem {
  uint8_t id;
  uint8_t size;
  HID_Report_Storage_t data;
};

static struct ReportItem queued[ REPORT_QUEUE_SIZE ];

bool HID_Report(uint8_t id, const void *data, uint8_t size)
{
  for(uint8_t i = 0; i < REPORT_QUEUE_SIZE; ++i) {
    if(queued[i].size == 0) {
      memcpy(&queued[i].data, data, size);
      queued[i].id = id;
      queued[i].size = size;
      return true;
    }
  }
  return false;
}

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Sensor_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber                = INTERFACE_ID_HID_SENSOR,
				.ReportINEndpoint               =
					{
						.Address                = SENSOR_EPADDR,
						.Size                   = SENSOR_EPSIZE,
						.Banks                  = 1,
					},
                .PrevReportINBufferSize         = SENSOR_EPSIZE,
			},
	};

void HID_Task()
{
   HID_Device_USBTask(&Sensor_HID_Interface);
}

bool HID_EVENT_USB_Device_ConfigurationChanged(void)
{
  bool ConfigSuccess = true;
  
  ConfigSuccess &= HID_Device_ConfigureEndpoints(&Sensor_HID_Interface);

  return ConfigSuccess;
}

void HID_EVENT_USB_Device_ControlRequest(void)
{
  HID_Device_ProcessControlRequest(&Sensor_HID_Interface);
}

void HID_EVENT_USB_Device_StartOfFrame(void)
{
  HID_Device_MillisecondElapsed(&Sensor_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
    bool ret = false;
    
    switch(ReportType) {
        case HID_REPORT_ITEM_In:
            *ReportSize = 0;
            for(uint8_t i = 0; i < REPORT_QUEUE_SIZE; ++i) {
              if(queued[i].size && ((*ReportID == 0) || (*ReportID == queued[i].id))) {
                memcpy(ReportData, &queued[i].data, queued[i].size);
                *ReportSize = queued[i].size;
                *ReportID = queued[i].id;
                queued[i].size = 0;
                ret = true;
                break;
              }
            }

            break;
        
        case HID_REPORT_ITEM_Feature: // unsupported
            break;
	}

	return ret;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
}
