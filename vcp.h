#ifndef _WASTE_OIL_BURNER_VCP_H_
#define _WASTE_OIL_BURNER_VCP_H_

#ifdef USE_USB_VCP
# include <LUFA/Drivers/USB/USB.h>
#endif

#include <avr/pgmspace.h>

void VCP_Init(void);

void VCP_Task(void);

int VCP_Read(uint8_t *buf, int size);
int VCP_Write(uint8_t *buf, int size);

#ifdef USE_USB_VCP
void EVENT_VCP_SetLineEncoding(CDC_LineEncoding_t *LineEncoding);
void EVENT_VCP_SetControlLineState(uint16_t State);
void EVENT_VCP_DataReceived(void);

void VCP_EVENT_USB_Device_ControlRequest(void);
bool VCP_EVENT_USB_Device_ConfigurationChanged(void);
#endif /* USE_USB_VCP */

#endif /* _WASTE_OIL_BURNER_VCP_H_ */
