#include "main.h"
#include "scheduler.h"

#include <stdio.h>
#include <stdint.h>

TIM_HandleTypeDef htim2;

#define UART_BAUD_RATE                  57600U
#define BUTTON_POLL_MS                  20U
#define TERMINAL_CLEAR_INTERVAL_MS      10000U
#define TERMINAL_VISIBLE_ROWS           30U

#define UART_TX_BUFFER_SIZE             512U
#define UART_TX_BUFFER_MASK              (UART_TX_BUFFER_SIZE - 1U)

#if ((UART_TX_BUFFER_SIZE & (UART_TX_BUFFER_SIZE - 1U)) != 0U)
#error "UART_TX_BUFFER_SIZE must be a power of two"
#endif

/* Task functions. */
static void Task_LED_D1(void);
static void Task_LED_D2(void);
static void Task_LED_D3(void);
static void Task_LED_D4(void);
static void Task_LED_D5(void);
static void Task_Button_Poll(void);
static void Task_OneShot_At_3s(void);
static void Task_Clear_Terminal_Every_10s(void);
static void Task_Report_Every_10ms(void);

/* Peripheral initialization functions. */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_TX_Init(void);

/* -------------------------------------------------------------------------- */
/* UART asynchronous TX ring buffer.                                          */
/* printf() only queues bytes; USART1 TXE interrupt sends them in background. */
/* This prevents terminal output from blocking the cooperative scheduler.    */
/* -------------------------------------------------------------------------- */
static uint8_t g_uartTxBuffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t g_uartTxHead = 0U;
static volatile uint16_t g_uartTxTail = 0U;

static void UART_Queue_Byte(uint8_t byte)
{
    uint16_t head = g_uartTxHead;
    uint16_t next = (uint16_t)((head + 1U) & UART_TX_BUFFER_MASK);

    /* Wait only if the queue is full. USART1 interrupt remains enabled and
     * drains the queue while foreground code waits. */
    while (next == g_uartTxTail)
    {
        /* The TXE interrupt advances g_uartTxTail. */
    }

    g_uartTxBuffer[head] = byte;
    __DMB();
    g_uartTxHead = next;
    USART1->CR1 |= USART_CR1_TXEIE;
}

int __io_putchar(int character)
{
    UART_Queue_Byte((uint8_t)character);
    return character;
}

/* Called by USART1_IRQHandler() from stm32f1xx_it.c. */
void UART_Tx_Empty_IRQHandler(void)
{
    if (((USART1->SR & USART_SR_TXE) != 0U) &&
        ((USART1->CR1 & USART_CR1_TXEIE) != 0U))
    {
        uint16_t tail = g_uartTxTail;

        if (tail != g_uartTxHead)
        {
            USART1->DR = g_uartTxBuffer[tail];
            g_uartTxTail = (uint16_t)((tail + 1U) & UART_TX_BUFFER_MASK);
        }
        else
        {
            /* Nothing left to transmit. Stop TX-empty interrupts. */
            USART1->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Event collection. All application callbacks run in the foreground, so the */
/* reporter can safely collect and clear these flags once per 10 ms tick.     */
/* -------------------------------------------------------------------------- */
#define LED_EVENT_D1       (1U << 0)
#define LED_EVENT_D2       (1U << 1)
#define LED_EVENT_D3       (1U << 2)
#define LED_EVENT_D4       (1U << 3)
#define LED_EVENT_D5       (1U << 4)

#define BUTTON_EVENT_NONE      0U
#define BUTTON_EVENT_PRESSED   1U
#define BUTTON_EVENT_RELEASED  2U

static uint8_t g_ledChangedMask = 0U;
static uint8_t g_ledOnMask = 0U;
static uint8_t g_buttonEvent = BUTTON_EVENT_NONE;
static uint8_t g_d6IsOn = 0U;
static uint8_t g_oneShotPending = 0U;
static uint8_t g_terminalClearedPending = 0U;

/* Button debounce state: PB0 is active HIGH with external pull-down. */
static GPIO_PinState g_buttonLastSample = GPIO_PIN_RESET;
static GPIO_PinState g_buttonStableState = GPIO_PIN_RESET;
static uint8_t g_buttonSameSampleCount = 0U;

static void Print_Task_Registration_Error(void)
{
    printf("[ERROR] Scheduler task registration failed.\r\n");
    Error_Handler();
}

/* Change an LED and record its final state. No UART output is done here. */
static void Toggle_LED_And_Collect(uint16_t pin, uint8_t eventBit)
{
    if ((g_ledOnMask & eventBit) != 0U)
    {
        g_ledOnMask &= (uint8_t)(~eventBit);
        HAL_GPIO_WritePin(LED_GPIO_PORT, pin, LED_OFF_STATE);
    }
    else
    {
        g_ledOnMask |= eventBit;
        HAL_GPIO_WritePin(LED_GPIO_PORT, pin, LED_ON_STATE);
    }

    g_ledChangedMask |= eventBit;
}

/* Push blank rows into the UART queue to scroll old text out of view.
 * The UART TX interrupt transmits these bytes asynchronously. */
static void Terminal_Clear_Visible_Area(void)
{
    uint8_t row;

    for (row = 0U; row < TERMINAL_VISIBLE_ROWS; row++)
    {
        UART_Queue_Byte('\r');
        UART_Queue_Byte('\n');
    }
}

int main(void)
{
    uint32_t ledD1TaskID;
    uint32_t ledD2TaskID;
    uint32_t ledD3TaskID;
    uint32_t ledD4TaskID;
    uint32_t ledD5TaskID;
    uint32_t buttonTaskID;
    uint32_t oneShotTaskID;
    uint32_t clearTerminalTaskID;
    uint32_t report10msTaskID;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_TX_Init();
    MX_TIM2_Init();

    SCH_Init();

    /* Five periodic LED tasks. Each callback only changes GPIO and records
     * the event; the 10 ms reporter prints all same-tick changes on one line. */
    ledD1TaskID = SCH_Add_Task(Task_LED_D1,
                               SCH_MS_TO_TICKS(500U),
                               SCH_MS_TO_TICKS(500U));
    ledD2TaskID = SCH_Add_Task(Task_LED_D2,
                               SCH_MS_TO_TICKS(1000U),
                               SCH_MS_TO_TICKS(1000U));
    ledD3TaskID = SCH_Add_Task(Task_LED_D3,
                               SCH_MS_TO_TICKS(1500U),
                               SCH_MS_TO_TICKS(1500U));
    ledD4TaskID = SCH_Add_Task(Task_LED_D4,
                               SCH_MS_TO_TICKS(2000U),
                               SCH_MS_TO_TICKS(2000U));
    ledD5TaskID = SCH_Add_Task(Task_LED_D5,
                               SCH_MS_TO_TICKS(2500U),
                               SCH_MS_TO_TICKS(2500U));

    /* Button polling is periodic at 20 ms. */
    buttonTaskID = SCH_Add_Task(Task_Button_Poll,
                                SCH_MS_TO_TICKS(BUTTON_POLL_MS),
                                SCH_MS_TO_TICKS(BUTTON_POLL_MS));

    /* One-shot event at exactly the 3-second scheduler deadline. */
    oneShotTaskID = SCH_Add_Task(Task_OneShot_At_3s,
                                 SCH_MS_TO_TICKS(3000U),
                                 0U);

    /* Clear the visible terminal every 10 seconds, first at t = 10 s. */
    clearTerminalTaskID = SCH_Add_Task(Task_Clear_Terminal_Every_10s,
                                       SCH_MS_TO_TICKS(TERMINAL_CLEAR_INTERVAL_MS),
                                       SCH_MS_TO_TICKS(TERMINAL_CLEAR_INTERVAL_MS));

    /* IMPORTANT: register the reporter LAST. The scheduler preserves FIFO
     * order for equal deadlines, so all same-tick LED/Button/One-shot/Clear
     * callbacks record their events before this task prints the 10 ms line. */
    report10msTaskID = SCH_Add_Task(Task_Report_Every_10ms,
                                    SCH_MS_TO_TICKS(10U),
                                    SCH_MS_TO_TICKS(10U));

    if ((ledD1TaskID == SCH_INVALID_TASK_ID) ||
        (ledD2TaskID == SCH_INVALID_TASK_ID) ||
        (ledD3TaskID == SCH_INVALID_TASK_ID) ||
        (ledD4TaskID == SCH_INVALID_TASK_ID) ||
        (ledD5TaskID == SCH_INVALID_TASK_ID) ||
        (buttonTaskID == SCH_INVALID_TASK_ID) ||
        (oneShotTaskID == SCH_INVALID_TASK_ID) ||
        (clearTerminalTaskID == SCH_INVALID_TASK_ID) ||
        (report10msTaskID == SCH_INVALID_TASK_ID) ||
        (SCH_Get_Last_Error() != SCH_OK))
    {
        Print_Task_Registration_Error();
    }

    /* LEDs are active LOW and start OFF. PB0 released = LOW. */
    g_ledOnMask = 0U;
    g_ledChangedMask = 0U;
    g_d6IsOn = 0U;
    HAL_GPIO_WritePin(LED_GPIO_PORT,
                      LED_D1_PIN | LED_D2_PIN | LED_D3_PIN |
                      LED_D4_PIN | LED_D5_PIN | LED_D6_PIN,
                      LED_OFF_STATE);

    g_buttonLastSample = HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN);
    g_buttonStableState = g_buttonLastSample;
    g_buttonSameSampleCount = 0U;

    printf("\r\n===== LAB 4: COOPERATIVE SCHEDULER =====\r\n");
    printf("TIM2 tick: 10 ms | USART1: 57600, 8-N-1\r\n");
    printf("D1=500 ms, D2=1000 ms, D3=1500 ms, D4=2000 ms, D5=2500 ms\r\n");
    printf("Button: PB0 -> D6 PA6 | One-shot: 3000 ms | Clear: 10000 ms\r\n");
    printf("One timestamp line is printed every 10 ms.\r\n\r\n");

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        printf("[ERROR] TIM2 could not be started.\r\n");
        Error_Handler();
    }

    while (1)
    {
        SCH_Dispatch_Tasks();

        if (SCH_Get_Last_Error() != SCH_OK)
        {
            printf("[ERROR] Scheduler error %u at %lu ms\r\n",
                   (unsigned int)SCH_Get_Last_Error(),
                   (unsigned long)get_time());
            SCH_Clear_Last_Error();
        }
    }
}

/* ---------------------------- Periodic LED tasks ------------------------- */

static void Task_LED_D1(void)
{
    Toggle_LED_And_Collect(LED_D1_PIN, LED_EVENT_D1);
}

static void Task_LED_D2(void)
{
    Toggle_LED_And_Collect(LED_D2_PIN, LED_EVENT_D2);
}

static void Task_LED_D3(void)
{
    Toggle_LED_And_Collect(LED_D3_PIN, LED_EVENT_D3);
}

static void Task_LED_D4(void)
{
    Toggle_LED_And_Collect(LED_D4_PIN, LED_EVENT_D4);
}

static void Task_LED_D5(void)
{
    Toggle_LED_And_Collect(LED_D5_PIN, LED_EVENT_D5);
}

/* ------------------------------ Button task ------------------------------ */

static void Task_Button_Poll(void)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN);

    if (raw == g_buttonLastSample)
    {
        if (g_buttonSameSampleCount < 2U)
        {
            g_buttonSameSampleCount++;
        }
    }
    else
    {
        g_buttonLastSample = raw;
        g_buttonSameSampleCount = 1U;
    }

    /* Two consecutive equal samples are required to accept the new state. */
    if ((g_buttonSameSampleCount >= 2U) &&
        (raw != g_buttonStableState))
    {
        g_buttonStableState = raw;

        if (g_buttonStableState == GPIO_PIN_SET)
        {
            g_d6IsOn = (uint8_t)(g_d6IsOn == 0U);
            HAL_GPIO_WritePin(LED_GPIO_PORT,
                              LED_D6_PIN,
                              (g_d6IsOn != 0U) ? LED_ON_STATE : LED_OFF_STATE);
            g_buttonEvent = BUTTON_EVENT_PRESSED;
        }
        else
        {
            g_buttonEvent = BUTTON_EVENT_RELEASED;
        }
    }
}

/* ------------------------------ One-shot --------------------------------- */

static void Task_OneShot_At_3s(void)
{
    /* The 10 ms reporter prints this event on the same timestamp line. */
    g_oneShotPending = 1U;
}

/* --------------------------- Terminal clear ------------------------------ */

static void Task_Clear_Terminal_Every_10s(void)
{
    Terminal_Clear_Visible_Area();
    g_terminalClearedPending = 1U;
}

/* ------------------------- 10 ms consolidated log ------------------------ */

static void Print_Separator(uint8_t *firstItem)
{
    if (*firstItem != 0U)
    {
        printf(" ");
        *firstItem = 0U;
    }
    else
    {
        printf(", ");
    }
}

static void Task_Report_Every_10ms(void)
{
    static const uint8_t ledBits[5] = {
        LED_EVENT_D1, LED_EVENT_D2, LED_EVENT_D3,
        LED_EVENT_D4, LED_EVENT_D5
    };
    static const char * const ledNames[5] = {"D1", "D2", "D3", "D4", "D5"};

    uint8_t firstItem = 1U;
    uint8_t i;
    uint8_t changedMask = g_ledChangedMask;
    uint8_t onMask = g_ledOnMask;
    uint8_t buttonEvent = g_buttonEvent;
    uint8_t oneShot = g_oneShotPending;
    uint8_t terminalCleared = g_terminalClearedPending;
    uint32_t timestamp = get_time();

    /* A timestamp row is emitted on EVERY scheduler tick (10 ms). Events
     * detected during this tick are appended to this same row. */
    printf("[%lums]", (unsigned long)timestamp);

    for (i = 0U; i < 5U; i++)
    {
        if ((changedMask & ledBits[i]) != 0U)
        {
            Print_Separator(&firstItem);
            printf("%s %s",
                   ledNames[i],
                   ((onMask & ledBits[i]) != 0U) ? "ON" : "OFF");
        }
    }

    if (buttonEvent == BUTTON_EVENT_PRESSED)
    {
        Print_Separator(&firstItem);
        printf("BUTTON PRESSED, D6 %s", (g_d6IsOn != 0U) ? "ON" : "OFF");
    }
    else if (buttonEvent == BUTTON_EVENT_RELEASED)
    {
        Print_Separator(&firstItem);
        printf("BUTTON RELEASED");
    }

    if (oneShot != 0U)
    {
        Print_Separator(&firstItem);
        printf("ONE-SHOT 3s");
    }

    if (terminalCleared != 0U)
    {
        Print_Separator(&firstItem);
        printf("TERMINAL CLEARED");
    }

    printf("\r\n");

    /* Clear only the events that were included in this row. All application
     * callbacks run serially in the foreground, so no event can be lost here. */
    g_ledChangedMask = 0U;
    g_buttonEvent = BUTTON_EVENT_NONE;
    g_oneShotPending = 0U;
    g_terminalClearedPending = 0U;
}

/* ------------------------- Timer / peripherals --------------------------- */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        SCH_Update();
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clock = {0};

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        Error_Handler();
    }

    clock.ClockType = RCC_CLOCKTYPE_HCLK |
                      RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 |
                      RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV1;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* LEDs PA1..PA6 are active LOW and are initialized OFF. */
    gpio.Pin = LED_D1_PIN | LED_D2_PIN | LED_D3_PIN |
               LED_D4_PIN | LED_D5_PIN | LED_D6_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &gpio);

    HAL_GPIO_WritePin(LED_GPIO_PORT,
                      LED_D1_PIN | LED_D2_PIN | LED_D3_PIN |
                      LED_D4_PIN | LED_D5_PIN | LED_D6_PIN,
                      LED_OFF_STATE);

    /* PB0 has an external pull-down resistor in the Proteus schematic. */
    gpio.Pin = BUTTON_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON_GPIO_PORT, &gpio);
}

static void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 7999U;       /* 8 MHz / 8000 = 1 kHz */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9U;             /* 10 counts = 10 ms */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART1_TX_Init(void)
{
    uint32_t pclk2;

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* USART1 default mapping: PA9 TX. MODE9=11, CNF9=10 => nibble 0xB. */
    AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;
    GPIOA->CRH = (GPIOA->CRH & ~(0x0FUL << 4U)) | (0x0BUL << 4U);
    GPIOA->BSRR = UART_TX_PIN;

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;

    pclk2 = HAL_RCC_GetPCLK2Freq();
    /* At PCLK2=8 MHz and 57600 baud, BRR rounds to 139. */
    USART1->BRR = (uint16_t)((pclk2 + (UART_BAUD_RATE / 2U)) /
                             UART_BAUD_RATE);

    USART1->CR1 = USART_CR1_TE | USART_CR1_UE; /* TX only, 8-N-1. */

    /* USART1 TX-empty interrupt drains the UART ring buffer. */
    HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        /* Stop here if initialization or task registration fails. */
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
