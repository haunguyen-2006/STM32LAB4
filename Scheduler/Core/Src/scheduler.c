#include "main.h"
#include "scheduler.h"

#include <stddef.h>
#include <string.h>

#define SCH_NO_SLOT             0xFFU
#define SCH_READY_QUEUE_SIZE    SCH_MAX_TASKS
#define SCH_MAX_INTERVAL_TICKS  0x7FFFFFFFUL

typedef struct
{
    SCH_TaskFunction_t function;
    uint32_t id;
    uint32_t dueTick;
    uint32_t periodTicks;
    uint8_t nextDeadline;
    uint8_t active;
    uint8_t inDeadlineList;
    uint8_t inReadyQueue;
    uint8_t running;
} SCH_Task_t;

static SCH_Task_t g_tasks[SCH_MAX_TASKS];
static uint8_t g_deadlineHead = SCH_NO_SLOT;

static uint8_t g_readyQueue[SCH_READY_QUEUE_SIZE];
static uint8_t g_readyHead = 0U;
static uint8_t g_readyTail = 0U;
static uint8_t g_readyCount = 0U;

static volatile uint32_t g_tickCount = 0U;
static uint32_t g_nextTaskID = 1U;
static volatile SCH_Error_t g_lastError = SCH_OK;

/* Preserve the previous interrupt mask when entering a critical section. */
static uint32_t SCH_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void SCH_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/* Wrap-safe tick comparisons. All intervals are limited to < 2^31 ticks. */
static uint8_t SCH_TimeReached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static uint8_t SCH_TimeBefore(uint32_t a, uint32_t b)
{
    return ((int32_t)(a - b) < 0) ? 1U : 0U;
}

static void SCH_ClearSlot(uint8_t slot)
{
    memset(&g_tasks[slot], 0, sizeof(g_tasks[slot]));
    g_tasks[slot].nextDeadline = SCH_NO_SLOT;
}

/* Keep the deadline list sorted from earliest to latest due tick.
 * This insertion is done in foreground context, never in the TIM2 ISR. */
static void SCH_InsertDeadline(uint8_t slot)
{
    uint8_t previous = SCH_NO_SLOT;
    uint8_t current = g_deadlineHead;
    uint32_t newDeadline = g_tasks[slot].dueTick;

    while (current != SCH_NO_SLOT)
    {
        /* Use strict comparison so equal deadlines retain FIFO order. */
        if (SCH_TimeBefore(newDeadline, g_tasks[current].dueTick) != 0U)
        {
            break;
        }

        previous = current;
        current = g_tasks[current].nextDeadline;
    }

    g_tasks[slot].nextDeadline = current;
    g_tasks[slot].inDeadlineList = 1U;

    if (previous == SCH_NO_SLOT)
    {
        g_deadlineHead = slot;
    }
    else
    {
        g_tasks[previous].nextDeadline = slot;
    }
}

static void SCH_RemoveDeadline(uint8_t slot)
{
    uint8_t previous = SCH_NO_SLOT;
    uint8_t current = g_deadlineHead;

    while (current != SCH_NO_SLOT)
    {
        if (current == slot)
        {
            if (previous == SCH_NO_SLOT)
            {
                g_deadlineHead = g_tasks[current].nextDeadline;
            }
            else
            {
                g_tasks[previous].nextDeadline =
                    g_tasks[current].nextDeadline;
            }

            g_tasks[current].nextDeadline = SCH_NO_SLOT;
            g_tasks[current].inDeadlineList = 0U;
            return;
        }

        previous = current;
        current = g_tasks[current].nextDeadline;
    }

    g_tasks[slot].nextDeadline = SCH_NO_SLOT;
    g_tasks[slot].inDeadlineList = 0U;
}

/* Called from SCH_Update() in interrupt context. */
static uint8_t SCH_EnqueueReadyFromISR(uint8_t slot)
{
    if (g_tasks[slot].inReadyQueue != 0U)
    {
        return 1U;
    }

    if (g_readyCount >= SCH_READY_QUEUE_SIZE)
    {
        g_lastError = SCH_ERROR_READY_QUEUE_OVERFLOW;
        return 0U;
    }

    g_readyQueue[g_readyTail] = slot;
    g_readyTail = (uint8_t)((g_readyTail + 1U) % SCH_READY_QUEUE_SIZE);
    g_readyCount++;
    g_tasks[slot].inReadyQueue = 1U;
    return 1U;
}

static uint8_t SCH_DequeueReady(void)
{
    uint8_t slot = g_readyQueue[g_readyHead];
    g_readyHead = (uint8_t)((g_readyHead + 1U) % SCH_READY_QUEUE_SIZE);
    g_readyCount--;
    g_tasks[slot].inReadyQueue = 0U;
    return slot;
}

static void SCH_RemoveReadyEntries(uint8_t slot)
{
    uint8_t temp[SCH_READY_QUEUE_SIZE];
    uint8_t tempCount = 0U;
    uint8_t i;

    for (i = 0U; i < g_readyCount; i++)
    {
        uint8_t index =
            (uint8_t)((g_readyHead + i) % SCH_READY_QUEUE_SIZE);
        uint8_t queuedSlot = g_readyQueue[index];

        if (queuedSlot != slot)
        {
            temp[tempCount++] = queuedSlot;
        }
    }

    for (i = 0U; i < tempCount; i++)
    {
        g_readyQueue[i] = temp[i];
    }

    g_readyHead = 0U;
    g_readyTail = (uint8_t)(tempCount % SCH_READY_QUEUE_SIZE);
    g_readyCount = tempCount;
    g_tasks[slot].inReadyQueue = 0U;
}

void SCH_Init(void)
{
    uint32_t primask = SCH_EnterCritical();
    uint8_t i;

    memset(g_tasks, 0, sizeof(g_tasks));
    memset(g_readyQueue, 0, sizeof(g_readyQueue));

    for (i = 0U; i < SCH_MAX_TASKS; i++)
    {
        g_tasks[i].nextDeadline = SCH_NO_SLOT;
    }

    g_deadlineHead = SCH_NO_SLOT;
    g_readyHead = 0U;
    g_readyTail = 0U;
    g_readyCount = 0U;
    g_tickCount = 0U;
    g_nextTaskID = 1U;
    g_lastError = SCH_OK;

    SCH_ExitCritical(primask);
}

uint32_t SCH_Add_Task(SCH_TaskFunction_t function,
                      uint32_t delayTicks,
                      uint32_t periodTicks)
{
    uint32_t primask;
    uint32_t taskID;
    uint8_t slot;

    if ((function == NULL) ||
        (delayTicks > SCH_MAX_INTERVAL_TICKS) ||
        (periodTicks > SCH_MAX_INTERVAL_TICKS))
    {
        g_lastError = SCH_ERROR_INVALID_ARGUMENT;
        return SCH_INVALID_TASK_ID;
    }

    primask = SCH_EnterCritical();

    for (slot = 0U; slot < SCH_MAX_TASKS; slot++)
    {
        if (g_tasks[slot].active == 0U)
        {
            break;
        }
    }

    if (slot >= SCH_MAX_TASKS)
    {
        g_lastError = SCH_ERROR_NO_TASK_SLOT;
        SCH_ExitCritical(primask);
        return SCH_INVALID_TASK_ID;
    }

    taskID = g_nextTaskID++;
    if (taskID == SCH_INVALID_TASK_ID)
    {
        taskID = g_nextTaskID++;
    }

    g_tasks[slot].function = function;
    g_tasks[slot].id = taskID;
    g_tasks[slot].dueTick = g_tickCount + delayTicks;
    g_tasks[slot].periodTicks = periodTicks;
    g_tasks[slot].nextDeadline = SCH_NO_SLOT;
    g_tasks[slot].active = 1U;
    g_tasks[slot].inDeadlineList = 0U;
    g_tasks[slot].inReadyQueue = 0U;
    g_tasks[slot].running = 0U;

    SCH_InsertDeadline(slot);
    SCH_ExitCritical(primask);

    return taskID;
}

/* Called by the TIM2 interrupt callback every 10 ms.
 * It updates time and moves due tasks to the ready FIFO. It does not run
 * application callbacks and does not perform UART or GPIO logging here. */
void SCH_Update(void)
{
    g_tickCount++;

    while (g_deadlineHead != SCH_NO_SLOT)
    {
        uint8_t slot = g_deadlineHead;

        if (SCH_TimeReached(g_tickCount, g_tasks[slot].dueTick) == 0U)
        {
            break;
        }

        /* If the ready queue is unexpectedly full, keep this due task at the
         * head and retry on the next tick instead of silently losing it. */
        if (g_readyCount >= SCH_READY_QUEUE_SIZE)
        {
            g_lastError = SCH_ERROR_READY_QUEUE_OVERFLOW;
            break;
        }

        g_deadlineHead = g_tasks[slot].nextDeadline;
        g_tasks[slot].nextDeadline = SCH_NO_SLOT;
        g_tasks[slot].inDeadlineList = 0U;

        if ((g_tasks[slot].active != 0U) &&
            (g_tasks[slot].inReadyQueue == 0U))
        {
            (void)SCH_EnqueueReadyFromISR(slot);
        }
    }
}

void SCH_Dispatch_Tasks(void)
{
    for (;;)
    {
        uint32_t primask = SCH_EnterCritical();
        uint8_t slot;
        uint32_t taskID;
        uint32_t periodTicks;
        uint32_t scheduledDeadline;
        uint32_t now;
        SCH_TaskFunction_t function;

        if (g_readyCount == 0U)
        {
            SCH_ExitCritical(primask);
            break;
        }

        slot = SCH_DequeueReady();
        if ((slot >= SCH_MAX_TASKS) || (g_tasks[slot].active == 0U))
        {
            SCH_ExitCritical(primask);
            continue;
        }

        g_tasks[slot].running = 1U;
        function = g_tasks[slot].function;
        taskID = g_tasks[slot].id;
        periodTicks = g_tasks[slot].periodTicks;
        scheduledDeadline = g_tasks[slot].dueTick;

        /* Re-arm periodic tasks before executing their callbacks. The next
         * deadline is based on the previous schedule, avoiding drift. */
        if (periodTicks > 0U)
        {
            uint32_t nextDeadline = scheduledDeadline + periodTicks;
            now = g_tickCount;

            /* If dispatch was delayed, skip missed periods and schedule the
             * first future deadline rather than executing a burst of catch-up
             * callbacks. */
            if (SCH_TimeReached(now, nextDeadline) != 0U)
            {
                uint32_t overdue = now - nextDeadline;
                uint32_t missed = (overdue / periodTicks) + 1U;
                nextDeadline += missed * periodTicks;
            }

            g_tasks[slot].dueTick = nextDeadline;
            SCH_InsertDeadline(slot);
        }

        SCH_ExitCritical(primask);

        /* Run application code in foreground context, never inside the ISR. */
        if (function != NULL)
        {
            function();
        }

        primask = SCH_EnterCritical();
        if ((g_tasks[slot].active != 0U) && (g_tasks[slot].id == taskID))
        {
            g_tasks[slot].running = 0U;
            if (periodTicks == 0U)
            {
                SCH_ClearSlot(slot);
            }
        }
        SCH_ExitCritical(primask);
    }
}

uint8_t SCH_Delete_Task(uint32_t taskID)
{
    uint32_t primask;
    uint8_t slot;

    if (taskID == SCH_INVALID_TASK_ID)
    {
        g_lastError = SCH_ERROR_INVALID_TASK_ID;
        return 0U;
    }

    primask = SCH_EnterCritical();

    for (slot = 0U; slot < SCH_MAX_TASKS; slot++)
    {
        if ((g_tasks[slot].active != 0U) && (g_tasks[slot].id == taskID))
        {
            break;
        }
    }

    if (slot >= SCH_MAX_TASKS)
    {
        g_lastError = SCH_ERROR_INVALID_TASK_ID;
        SCH_ExitCritical(primask);
        return 0U;
    }

    if (g_tasks[slot].inDeadlineList != 0U)
    {
        SCH_RemoveDeadline(slot);
    }

    if (g_tasks[slot].inReadyQueue != 0U)
    {
        SCH_RemoveReadyEntries(slot);
    }

    SCH_ClearSlot(slot);
    SCH_ExitCritical(primask);
    return 1U;
}

uint32_t SCH_Get_Tick_Count(void)
{
    uint32_t primask = SCH_EnterCritical();
    uint32_t ticks = g_tickCount;
    SCH_ExitCritical(primask);
    return ticks;
}

uint32_t get_time(void)
{
    return SCH_Get_Tick_Count() * SCH_TICK_MS;
}

SCH_Error_t SCH_Get_Last_Error(void)
{
    return g_lastError;
}

void SCH_Clear_Last_Error(void)
{
    g_lastError = SCH_OK;
}
