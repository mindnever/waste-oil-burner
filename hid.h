#ifndef _32U4IR_HID_H_
#define _32U4IR_HID_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef USE_USB_HID
# include <LUFA/Common/Common.h>
#endif

void HID_Task(void);

typedef struct {
	uint16_t SensorGUID;
	int16_t Temperature;
	uint8_t Humidity;
	uint8_t Battery;
} __attribute__((__packed__)) HID_RfRx_Report_02_t;

#define WOB_REPORT_INPUT_BUTTON_A (1 << 0)
#define WOB_REPORT_INPUT_BUTTON_B (1 << 1)
#define WOB_REPORT_INPUT_BURNING  (1 << 2)
#define WOB_REPORT_INPUT_BUTTON_R (1 << 3)

#define WOB_REPORT_OUTPUT_HEATER  (1 << 0)
#define WOB_REPORT_OUTPUT_FAN     (1 << 1)
#define WOB_REPORT_OUTPUT_AIR     (1 << 2)
#define WOB_REPORT_OUTPUT_SPARK   (1 << 3)
#define WOB_REPORT_OUTPUT_EXT1    (1 << 4)
#define WOB_REPORT_OUTPUT_EXT2    (1 << 5)
#define WOB_REPORT_OUTPUT_EXT3    (1 << 6)
#define WOB_REPORT_OUTPUT_EXT4    (1 << 7)

typedef struct {
	uint8_t State;
	uint8_t Flame;
	uint16_t OilTemperature;
	uint16_t WaterTemperature;
	uint8_t Inputs;
	uint8_t Outputs;
} __attribute__((__packed__)) HID_WOB_Report_03_t;

#define WOB_REPORT_FLAGS_CONTROL_ENABLED (1 << 0)
#define WOB_REPORT_FLAGS_OUTPUT_ACTIVE   (1 << 1)
#define WOB_REPORT_FLAGS_TIMEOUT_ALERT   (1 << 2)

#define WOB_REPORT_ZONE_INTERNAL_WATER   0
#define WOB_REPORT_ZONE_INTERNAL_OIL     1
#define WOB_REPORT_ZONE_EXTERNAL1        2
#define WOB_REPORT_ZONE_EXTERNAL2        3
#define WOB_REPORT_ZONE_EXTERNAL3        4
#define WOB_REPORT_ZONE_EXTERNAL4        5

#define _WOB_REPORT_ZONE_COUNT           6

typedef struct {
	uint8_t Zone;
	uint8_t Flags;
	int16_t SetPoint;
	int16_t Current;
} __attribute__((__packed__)) HID_WOB_Report_04_t;

typedef struct {
	uint8_t Zone;
	int16_t SetPoint;
	uint16_t Hysteresis;
	uint8_t SensorType;
	uint16_t SensorID;
} __attribute__((__packed__)) HID_WOB_Report_05_t; // Zone configuration

typedef union {
	HID_RfRx_Report_02_t r02;
	HID_WOB_Report_03_t r03;
	HID_WOB_Report_04_t r04;
	HID_WOB_Report_05_t r05;
} HID_Report_Storage_t;

bool HID_Report(uint8_t reportId, const void *reportData, uint8_t reportSize);

#ifdef USE_USB_HID

bool HID_EVENT_USB_Device_ConfigurationChanged(void);
void HID_EVENT_USB_Device_ControlRequest(void);
void HID_EVENT_USB_Device_StartOfFrame(void);

#endif

#endif /* _32U4IR_HID_H_ */
