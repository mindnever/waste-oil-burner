#ifndef _32U4IR_H_
#define _32U4IR_H_

#include <stdbool.h>
#include <LUFA/Common/Common.h>

void HID_Task(void);

typedef struct
{
    uint8_t State;
    uint8_t Event;
    uint16_t SensorGUID;
    uint16_t Temperature;
} ATTR_PACKED USB_Sensor_Data_t;

void HID_Report(uint8_t id, USB_Sensor_Data_t *report);

bool HID_EVENT_USB_Device_ConfigurationChanged(void);
void HID_EVENT_USB_Device_ControlRequest(void);
void HID_EVENT_USB_Device_StartOfFrame(void);

#endif /* _32U4IR_H_ */
