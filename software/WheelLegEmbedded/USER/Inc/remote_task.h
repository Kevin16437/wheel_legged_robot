#ifndef _REMOTE_TASK_H
#define _REMOTE_TASK_H

#include "start_init_task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t channels[5];
} RemoteRec;

extern TaskHandle_t Remote_Task_handler;
void Remote_Task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif
