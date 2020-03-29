/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

 /******************************************************************************
  * NOTE: Windows will not be running the FreeRTOS demo threads continuously, so
  * do not expect to get real time behaviour from the FreeRTOS Windows port, or
  * this demo application.  Also, the timing information in the FreeRTOS+Trace
  * logs have no meaningful units.  See the documentation page for the Windows
  * port for further information:
  * http://www.freertos.org/FreeRTOS-Windows-Simulator-Emulator-for-Visual-Studio-and-Eclipse-MingW.html
  *
  * NOTE 2:  This project provides two demo applications.  A simple blinky style
  * project, and a more comprehensive test and demo application.  The
  * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
  * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
  * in main.c.  This file implements the simply blinky version.  Console output
  * is used in place of the normal LED toggling.
  *
  * NOTE 3:  This file only contains the source code that is specific to the
  * basic demo.  Generic functions, such FreeRTOS hook functions, are defined
  * in main.c.
  ******************************************************************************
  *
  * main_blinky() creates one queue, one software timer, and two tasks.  It then
  * starts the scheduler.
  *
  * The Queue Send Task:
  * The queue send task is implemented by the prvQueueSendTask() function in
  * this file.  It uses vTaskDelayUntil() to create a periodic task that sends
  * the value 100 to the queue every 200 milliseconds (please read the notes
  * above regarding the accuracy of timing under Windows).
  *
  * The Queue Send Software Timer:
  * The timer is a one-shot timer that is reset by a key press.  The timer's
  * period is set to two seconds - if the timer expires then its callback
  * function writes the value 200 to the queue.  The callback function is
  * implemented by prvQueueSendTimerCallback() within this file.
  *
  * The Queue Receive Task:
  * The queue receive task is implemented by the prvQueueReceiveTask() function
  * in this file.  prvQueueReceiveTask() waits for data to arrive on the queue.
  * When data is received, the task checks the value of the data, then outputs a
  * message to indicate if the data came from the queue send task or the queue
  * send software timer.
  *
  * Expected Behaviour:
  * - The queue send task writes to the queue every 200ms, so every 200ms the
  *   queue receive task will output a message indicating that data was received
  *   on the queue from the queue send task.
  * - The queue send software timer has a period of two seconds, and is reset
  *   each time a key is pressed.  So if two seconds expire without a key being
  *   pressed then the queue receive task will output a message indicating that
  *   data was received on the queue from the queue send software timer.
  *
  * NOTE:  Console input and output relies on Windows system calls, which can
  * interfere with the execution of the FreeRTOS Windows port.  This demo only
  * uses Windows system call occasionally.  Heavier use of Windows system calls
  * can crash the port.
  */

  /* Standard includes. */
#include <stdio.h>
#include <conio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The times are converted from
milliseconds to ticks using the pdMS_TO_TICKS() macro. */
#define mainTASK_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 200UL )
#define mainTIMER_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 2000UL )

/* The number of items the queue can hold at once. */
#define mainQUEUE_LENGTH					( 2 )

/* The values sent to the queue receive task from the queue send task and the
queue send software timer respectively. */
#define mainVALUE_SENT_FROM_TASK			( 100UL )
#define mainVALUE_SENT_FROM_TIMER			( 200UL )

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/*-----------------------------------------------------------*/

/*** SEE THE COMMENTS AT THE TOP OF THIS FILE ***/
static uint32_t WorkFuncOne();
static uint32_t WorkFuncTwo();
static uint32_t WorkFuncThree();
static uint32_t HaltFuncOne();
static uint32_t HaltFuncTwo();
static uint32_t HaltFuncThree();
static uint32_t do_primes();
#define MAX_PRIME 1000
static uint32_t maxPrimes = MAX_PRIME;
typedef struct TaskAttr_s {
    uint32_t(*RunBeforeStart)();
    uint32_t(*WorkFunc)();
    uint32_t(*HaltFunc)();
    const char* TaskName;
    volatile uint32_t Priority;
    volatile uint32_t Cycles;
    volatile uint32_t WorkTime;
    volatile uint32_t HaltTime;
    volatile uint32_t Delay;
    volatile uint32_t Result;
    volatile uint32_t TestStages;
    TaskHandle_t TaskHandle;
    QueueHandle_t QueueHandle;
}TaskAttr_t;
static TaskHandle_t factoryTaskHandle;
static TaskAttr_t Attrs[3] = {
    {
        .RunBeforeStart = NULL,
        .WorkFunc = WorkFuncOne,
        .HaltFunc = HaltFuncOne,
        .TaskName = (char *)&"BATTERY",
        .Priority = 0,
        .Cycles = 0,
        .WorkTime = 0,
        .HaltTime = 0,
        .Delay = 0,
        .Result = 0,
        .TestStages = 1,
        .TaskHandle = NULL,
        .QueueHandle = NULL
    },
    {
        .RunBeforeStart = do_primes,
        .WorkFunc = WorkFuncTwo,
        .HaltFunc = HaltFuncTwo,
        .TaskName = (char *)&"CPU",
        .Priority = 1,
        .Cycles = 44,
        .WorkTime = pdMS_TO_TICKS(6 * 1000UL),
        .HaltTime = pdMS_TO_TICKS(9 * 1000UL),
        .Delay = 0,
        .Result = 0,
        .TestStages = 3,
        .TaskHandle = NULL,
        .QueueHandle = NULL
    },
    {
        .RunBeforeStart = NULL,
        .WorkFunc = WorkFuncThree,
        .HaltFunc = HaltFuncThree,
        .TaskName = (char *)&"MOTOR",
        .Priority = 1,
        .Cycles = 44,
        .WorkTime = pdMS_TO_TICKS(3000UL),
        .HaltTime = pdMS_TO_TICKS(1000UL),
        .Delay = 0,
        .Result = 0,
        .TestStages = 2,
        .TaskHandle = NULL,
        .QueueHandle = NULL
    }
};

static char queryBuffer[256];
typedef enum {
    TASK_NOT_STARTED,
    TASK_WORKING,
    TASK_HALTING,
    TASK_DONE,
}TaskStatus;

#define TASK_QUEUE_RCV_CMD_START        (0x5A51)
#define TASK_QUEUE_RCV_CMD_STOP         (0x5A52)
#define TASK_QUEUE_RCV_CMD_STAGE        (0x5A53)
#define TASK_QUEUE_RCV_CMD_QUERY        (0x5A54)
#define TASK_QUEUE_RCV_CMD_SUSPENDALL   (0x5A55)
#define TASK_QUEUE_RCV_CMD_SHIFT        (16)
#define TASK_QUEUE_RCV_STAGE_ONE        (0x01)
#define TASK_ALWAYS_RUN_PRIORITY_ZERO   (pdTRUE)

#include <time.h>  

void dsptime(const struct tm* ptm)
{
    printf("%d-%d-%d ", (ptm->tm_year + 1900), (ptm->tm_mon + 1), ptm->tm_mday);
    printf("%d:%d:%d \n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
};
const TickType_t xCPuMaxBlockTime = pdMS_TO_TICKS(9000);
static int CPUTest()
{
    time_t nowtime;
    nowtime = time(NULL);
    struct tm local;
    localtime_s(&local, &nowtime);
    printf("test start time: ");
    dsptime(&local);

    TickType_t xStartTicks = xTaskGetTickCount();
    TickType_t xTicksElapsed = 0;
    TickType_t xTicksToRun = pdMS_TO_TICKS(6000);
    TickType_t xWakeUpTime = xTaskGetTickCount();
   // printf("xStartTicks:%u \n", xStartTicks);
    // 0. CPU RUNNING
    //printf("test Started. \n");
    for (;;)
    {
        if (xTicksElapsed >= xTicksToRun)
        {
            nowtime = time(NULL);
            localtime_s(&local, &nowtime);
            printf("test done time: ");
            dsptime(&local);
            break;
        }
        do_primes();
        xTicksElapsed = xTaskGetTickCount() - xStartTicks;
    }
    xWakeUpTime = xTaskGetTickCount();
    vTaskDelayUntil(&xWakeUpTime, xCPuMaxBlockTime);
    nowtime = time(NULL);
    localtime_s(&local, &nowtime);
    printf("test stop time: ");
    dsptime(&local);
    //printf("test Stopped. \n ");
   // printf("xStopTicks:%u \n", xTicksRun);
}
static uint32_t do_primes()
{
    int start = 2, primes = 0;

    for (start = 2; start < maxPrimes; start++)
    {
        if (maxPrimes % start == 0)
        {
            primes++;
            break;
        }
    }

    return primes;
}
static uint32_t do_primes2()
{
    unsigned long i, num, primes = 0;
    for (num = 1; num <= maxPrimes; ++num) {
        for (i = 2; (i <= num) && (num % i != 0); ++i);
        if (i == num)
            ++primes;
    }

    maxPrimes = primes;

    return primes;
}

static uint32_t WorkFuncOne()
{
    //do_primes(MAX_PRIME);

    return 0;
}

static uint32_t WorkFuncTwo()
{
    do_primes();
    return 0;
}
static uint32_t WorkFuncThree()
{
    return 0;
}
static uint32_t HaltFuncOne()
{
    return 0;
}
static uint32_t HaltFuncTwo()
{
    return 0;
}

static uint32_t HaltFuncThree()
{
    return 0;
}

static void prvQueueWorkers(void* pvParameters)
{
    TaskAttr_t* pAttr;
    static TickType_t ticksWhenStageStart;
    static TickType_t ticksElapsedTicks;
    static int workerStats = 0;
    pAttr = (TaskAttr_t*)pvParameters;

    printf("TASK:%s Ready Start To Run delay time:%d!\n", pAttr->TaskName, pAttr->Delay);
    ticksWhenStageStart = xTaskGetTickCount();
    ticksElapsedTicks = 0;
    for (;; )
    {
        if (pAttr->Cycles > 0)
        {
            if (ticksElapsedTicks < pAttr->WorkTime)
            {
                if (pAttr->WorkFunc)
                    pAttr->WorkFunc(xTaskGetTickCount());
            }
            else
            {
                ticksWhenStageStart = xTaskGetTickCount();
                ticksElapsedTicks = 0;
                pAttr->Cycles--;
                continue;
            }
        }
        else
        {
            // work done.
        }

        ticksElapsedTicks += xTaskGetTickCount() - ticksWhenStageStart;
    }
}

static void prvLoadWorkers(uint32_t priority)
{
    TaskAttr_t* pAttr;
    uint32_t i;
    uint32_t casesCount;

    i = 0;
    pAttr = NULL;
    casesCount = sizeof(Attrs) / sizeof(Attrs[0]);

    for (i = 0; i < casesCount; i++)
    {
        if (priority == Attrs[i].Priority)
        {
            pAttr = &Attrs[i];

            pAttr->QueueHandle = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));
            if (pAttr->QueueHandle == NULL)
            {
                // put error code here.
                pAttr->Result = 0xfe;
                return;
            }


            if (pdTRUE != xTaskCreate(prvQueueWorkers, pAttr->TaskName
                , configMINIMAL_STACK_SIZE, (void *)(pAttr)
                , mainQUEUE_SEND_TASK_PRIORITY, &pAttr->TaskHandle))
            {
                pAttr->Result = 0xff;
                vQueueDelete(pAttr->QueueHandle);
                pAttr->QueueHandle = NULL;
                pAttr->TaskHandle = NULL;
                return;
            }
        }
    }
}

static void prvUnLoadWorkers(uint32_t priority)
{
    TaskAttr_t* pAttr;
    uint32_t i;
    uint32_t casesCount;

    i = 0;
    pAttr = NULL;
    casesCount = sizeof(Attrs) / sizeof(Attrs[0]);

    for (i = 0; i < casesCount; i++)
    {
        if (priority == Attrs[i].Priority)
        {
            pAttr = &Attrs[i];

            if (pAttr->QueueHandle != NULL)
            {
                vQueueDelete(pAttr->QueueHandle);
                pAttr->QueueHandle = NULL;
            }

            if (pAttr->TaskHandle != NULL)
            {
                vTaskDelete(pAttr->TaskHandle);
                pAttr->TaskHandle = NULL;
            }

            //TODO Set ERROR CODE.
        }
    }
}

static void prvFactoryTask(void* pvParameters)
{
    TickType_t xNextWakeTime;
    static TickType_t ticksCount = 0;
    TaskAttr_t* pAttr;
    uint32_t ulCases = sizeof(Attrs) / sizeof(Attrs[0]);
    uint32_t ulReceivedValue;
    uint32_t ulStage = 0xff;
    uint32_t ulStarted = 0xff;

    pAttr = (TaskAttr_t*)pvParameters;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();
    printf("Factory Task Ready Start To Run!\n");

    // 0. Run all caculation before task really start.
    while (ulCases)
    {
        if (Attrs[ulCases - 1].RunBeforeStart)
            Attrs[ulCases - 1].RunBeforeStart();
        ulCases--;
    }

    for (;; )
    {
        xQueueReceive(xQueue, &ulReceivedValue, portMAX_DELAY);

        /*****************************************************/
        /*             0. SET START                          */
        /*             1. SWITH STAGE                        */
        /*             2. SET STOP                           */
        /*****************************************************/
        if ((ulReceivedValue >> 16) == TASK_QUEUE_RCV_CMD_START)
        {
            ulStarted = pdTRUE;
            ulStage = 0;
            if (TASK_ALWAYS_RUN_PRIORITY_ZERO)
            {
                prvLoadWorkers(ulStage++);
                
            }
            prvLoadWorkers(ulStage);
        }
        else if ((ulReceivedValue >> 16) == TASK_QUEUE_RCV_CMD_STOP)
        {
            ulStarted = pdFALSE;
            if (TASK_ALWAYS_RUN_PRIORITY_ZERO)
            {
                prvUnLoadWorkers(0);
            }
            prvUnLoadWorkers(ulStage);
        }
        else if ((ulReceivedValue >> 16) == TASK_QUEUE_RCV_CMD_QUERY)
        {
            HeapStats_t stats;

            memset(queryBuffer, 0, sizeof(queryBuffer));
            vTaskList(queryBuffer);
            printf("%s\n", queryBuffer);
            vPortGetHeapStats(&stats);
            printf("xAvailableHeapSpaceInBytes:%d xMinimumEverFreeBytesRemaining:%d,"\
                "xNumberOfFreeBlocks:%d,xNumberOfSuccessfulAllocations:%d xNumberOfSuccessfulFrees:%d "\
                "xSizeOfLargestFreeBlockInBytes:%d xSizeOfSmallestFreeBlockInBytes:%d\n"
                , stats.xAvailableHeapSpaceInBytes, stats.xMinimumEverFreeBytesRemaining, stats.xNumberOfFreeBlocks
                , stats.xNumberOfSuccessfulAllocations, stats.xNumberOfSuccessfulFrees
                , stats.xSizeOfLargestFreeBlockInBytes, stats.xSizeOfSmallestFreeBlockInBytes);
        }
        else if ((ulReceivedValue >> 16) == TASK_QUEUE_RCV_CMD_STAGE)
        {
            ulStage = ulReceivedValue & 0xffff;
        }
        else
        {
            printf("Unexpected message\r\n");
        }
    }
}

static void prvDaemonTask(void* pvParameters)
{
    const TickType_t xBlockTime = mainTASK_SEND_FREQUENCY_MS;
    uint32_t cmd = 0;

    /* Prevent the compiler warning about the unused parameter. */
    (void)pvParameters;

    printf("Daemon Task Ready Start To Run!\n");

    for (;; )
    {
        vTaskDelay(xBlockTime);
        vTaskPrioritySet(xTaskGetCurrentTaskHandle(), configMAX_PRIORITIES - 1);
        CPUTest();
        if (_kbhit() != 0)
        {
            cmd = _getch();
            if (cmd == 's' || cmd == 'S')
            {
                cmd = TASK_QUEUE_RCV_CMD_START << TASK_QUEUE_RCV_CMD_SHIFT;
                xQueueSend(xQueue, &cmd, pdMS_TO_TICKS(100));
            }
            else if (cmd == 't' || cmd == 'T')
            {
                cmd = TASK_QUEUE_RCV_CMD_STOP << TASK_QUEUE_RCV_CMD_SHIFT;
                xQueueSend(xQueue, &cmd, pdMS_TO_TICKS(100));
            }
            else if (cmd == 'q' || cmd == 'Q')
            {
                cmd = TASK_QUEUE_RCV_CMD_QUERY << TASK_QUEUE_RCV_CMD_SHIFT;
                xQueueSend(xQueue, &cmd, pdMS_TO_TICKS(100));
            }
            else if (cmd >= '0' || cmd <= '9')
            {
                cmd = TASK_QUEUE_RCV_CMD_STAGE << TASK_QUEUE_RCV_CMD_SHIFT | atoi((char *)&cmd);
                xQueueSend(xQueue, &cmd, pdMS_TO_TICKS(100));
            }
        }
    }
}

void main_blinky(void)
{
    /* Create the queue. */
    xQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint32_t));

    if (xQueue != NULL)
    {
        // Factory Start To Work.
        xTaskCreate(prvFactoryTask, "Factory Task", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, &factoryTaskHandle);
        // Daemon Start To Work.
        xTaskCreate(prvDaemonTask, "Daemon Task", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL);

        vTaskStartScheduler();

    }

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the idle and/or
    timer tasks	to be created.  See the memory management section on the
    FreeRTOS web site for more details. */
    for (;; );
}
/*-----------------------------------------------------------*/


