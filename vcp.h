#ifndef _WASTE_OIL_BURNER_VCP_H_
#define _WASTE_OIL_BURNER_VCP_H_

#include <LUFA/Drivers/USB/USB.h>

void VCP_Task(void);

void VCP_Printf(const char *fmt, ...);

int VCP_Read(uint8_t *buf, int size);
int VCP_Write(uint8_t *buf, int size);

void VCP_Puts(const char *str);

void EVENT_VCP_SetLineEncoding(CDC_LineEncoding_t *LineEncoding);
void EVENT_VCP_SetControlLineState(uint16_t State);
void EVENT_VCP_DataReceived(void);

void VCP_EVENT_USB_Device_ControlRequest(void);
bool VCP_EVENT_USB_Device_ConfigurationChanged(void);

#endif /* _WASTE_OIL_BURNER_VCP_H_ */
