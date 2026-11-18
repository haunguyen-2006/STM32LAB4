#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCH_TICK_MS             10U
#define SCH_MAX_TASKS           16U
#define SCH_INVALID_TASK_ID     0U

/* Convert milliseconds to 10 ms scheduler ticks (round up). */
#define SCH_MS_TO_TICKS(ms) \
    (((uint32_t)(ms) + SCH_TICK_MS - 1U) / SCH_TICK_MS)

typedef void (*SCH_TaskFunction_t)(void);

typedef enum
{
    SCH_OK = 0,
    SCH_ERROR_INVALID_ARGUMENT,
    SCH_ERROR_NO_TASK_SLOT,
    SCH_ERROR_INVALID_TASK_ID,
    SCH_ERROR_READY_QUEUE_OVERFLOW
} SCH_Error_t;

void SCH_Init(void);

/* Called only from the TIM2 period-elapsed interrupt callback. */
void SCH_Update(void);

/* Called from the foreground while(1) loop; executes ready callbacks. */
void SCH_Dispatch_Tasks(void);

/* DELAY and PERIOD are in scheduler ticks, not milliseconds.
 * PERIOD == 0: one-shot task. PERIOD > 0: periodic task.
 * Returns a non-zero ID on success; otherwise SCH_INVALID_TASK_ID.
 */
uint32_t SCH_Add_Task(SCH_TaskFunction_t function,
                      uint32_t delayTicks,
                      uint32_t periodTicks);

/* Returns 1 if the task was found and deleted, otherwise 0. */
uint8_t SCH_Delete_Task(uint32_t taskID);

uint32_t SCH_Get_Tick_Count(void);
uint32_t get_time(void);
SCH_Error_t SCH_Get_Last_Error(void);
void SCH_Clear_Last_Error(void);

#ifdef __cplusplus
}
#endif

#endif /* __SCHEDULER_H */
