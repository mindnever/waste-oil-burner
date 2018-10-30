#include <LUFA/Drivers/USB/Class/Device/HIDClassDevice.h>
#include "hid.h"
#include "usb_descriptors.h"
#include "HidSensorSpec.h"
#include "vcp.h"

static bool report_queued = false;
static uint8_t report_id;
static USB_Sensor_Data_t queued_report;

static const uint8_t temp_sensor_feature_report[] =
{
  HID_SENSOR_VALUE_SIZE_32(100), /* report interval (ms) */
  HID_SENSOR_VALUE_SIZE_16(150 * 100), /* maximum sensor value (150C) */
  HID_SENSOR_VALUE_SIZE_16(-50 * 100), /* minimum sensor value (-50C) */
};

void HID_Report(uint8_t id, USB_Sensor_Data_t *r)
{
  if(!report_queued) {
    memcpy(&queued_report, r, sizeof(queued_report));
    report_id = id;
    report_queued = true;
  }
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
    switch(ReportType) {
        case HID_REPORT_ITEM_In:
            if(report_queued) {
                memcpy(ReportData, &queued_report, sizeof(queued_report));
                *ReportSize = sizeof(queued_report);
            
                report_queued = false;
            }
            break;
        
        case HID_REPORT_ITEM_Feature:
            memcpy(ReportData, &temp_sensor_feature_report, sizeof(temp_sensor_feature_report));
            *ReportSize = sizeof(temp_sensor_feature_report);
            break;
	}

	return true;
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
//  VCP_Printf("ProcessHIDReport: id %d, reportSize: %d\r\n", ReportID, ReportSize);
}
