#ifndef _32U4IR_HID_H_
#define _32U4IR_HID_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __AVR
# include <LUFA/Common/Common.h>
#endif

void HID_Task(void);

typedef struct
{
    uint16_t SensorGUID;
    uint16_t Temperature;
    uint8_t  Humidity;
} __attribute__((__packed__)) HID_RfRx_Report_02_t;

#define WOB_REPORT_INPUT_BUTTON_A (1 << 0)
#define WOB_REPORT_INPUT_BUTTON_B (1 << 1)
#define WOB_REPORT_INPUT_BURNING  (1 << 2)

#define WOB_REPORT_OUTPUT_HEATER (1 << 0)
#define WOB_REPORT_OUTPUT_FAN	 (1 << 1)
#define WOB_REPORT_OUTPUT_AIR    (1 << 2)
#define WOB_REPORT_OUTPUT_SPARK  (1 << 3)

typedef struct
{
    uint8_t State;
    uint8_t Flame;
    uint16_t OilTemperature;
    uint16_t WaterTemperature;
    uint8_t Inputs;
    uint8_t Outputs;
} __attribute__((__packed__)) HID_WOB_Report_03_t;

typedef union {
    HID_RfRx_Report_02_t r02;
    HID_WOB_Report_03_t r03;
} HID_Report_Storage_t;

#ifdef __AVR

void HID_Report(uint8_t reportId, const void *reportData, uint8_t reportSize);

bool HID_EVENT_USB_Device_ConfigurationChanged(void);
void HID_EVENT_USB_Device_ControlRequest(void);
void HID_EVENT_USB_Device_StartOfFrame(void);

#endif

#endif /* _32U4IR_HID_H_ */
