#ifndef _WOB_CLI_H_
#define _WOB_CLI_H_

void CLI_Init(void);
void CLI_Task(void);
void CLI_notify_P(const char *tag, const char *fmt, ...);
void CLI_Redraw(void);
void CLI_Erase(void);

#endif /* _WOB_CLI_H_ */
