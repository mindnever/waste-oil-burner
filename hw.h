#ifndef _WOB_HW_H_
#define _WOB_HW_H_

#if (BOARD == BOARD_PROMICRO)
# include "hw_promicro.h"
#elif (BOARD == BOARD_PROMINI)
# include "hw_promini.h"
#elif (BOARD == BOARD_UNO)
# include "hw_uno.h"
#endif

#if defined(ENCODER_A_PORT) && defined(ENCODER_B_PORT) && defined(BUTTON_R_PORT)
# define HAVE_ENCODER
#endif

#if defined(LCD_DATA_PORT)
# define LCD_COMMAND_DELAY_US 39
# define LCD_DATA_DELAY_US    43
#endif

#define TICK_MS               10

#endif /* _WOB_HW_H_ */
